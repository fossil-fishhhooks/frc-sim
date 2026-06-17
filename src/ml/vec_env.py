"""
vec_env.py — Thread-based vectorised env wrapper.

Each env is constructed AND run in its own thread so all NT servers
start simultaneously (fixing the sequential-init connection timeout).
"""

import threading
import time
import numpy as np

from env    import FRCSimEnv
from config import Config


class _EnvWorker:
    """Constructs and owns one FRCSimEnv entirely inside a dedicated thread."""

    def __init__(self, cfg: Config, port: int, env_class):
        self._cfg       = cfg
        self._port      = port
        self._env_class = env_class

        self.env: FRCSimEnv | None = None   # set by worker thread after init

        self._action: np.ndarray | None = None
        self._do_reset = False

        self.obs    = None
        self.reward = 0.0
        self.term   = False
        self.trunc  = False
        self.info   = {}
        self.error: Exception | None = None

        # init_done is set once the env is constructed (or failed).
        self._init_done    = threading.Event()
        self._action_ready = threading.Event()
        self._result_ready = threading.Event()
        self._stop         = threading.Event()

        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"env-{port}")
        self._thread.start()

    # ── called from main thread ───────────────────────────────────────────────

    def wait_init(self, timeout: float = 60.0):
        """Block until the env has been constructed in its thread."""
        if not self._init_done.wait(timeout):
            raise RuntimeError(f"env port={self._port} failed to init within {timeout}s")
        if self.error is not None:
            raise self.error

    def send_step(self, action: np.ndarray):
        self._do_reset = False
        self._action   = action
        self._result_ready.clear()
        self._action_ready.set()

    def send_reset(self):
        self._do_reset = True
        self._result_ready.clear()
        self._action_ready.set()

    def recv(self):
        self._result_ready.wait()
        if self.error is not None:
            raise self.error
        return self.obs, self.reward, self.term, self.trunc, self.info

    def connected_count(self) -> int:
        if self.env is None:
            return 0
        return self.env.connected_count()

    def stop(self):
        self._stop.set()
        self._action_ready.set()
        self._thread.join(timeout=2.0)
        if self.env is not None:
            self.env.close()

    # ── worker thread ─────────────────────────────────────────────────────────

    def _run(self):
        # Construct the env inside the thread so NT server binds in parallel.
        try:
            self.env   = self._env_class(self._cfg, port=self._port)
            self.error = None
        except Exception as e:
            print(f"  [VecEnv] port={self._port} init failed: {e}", flush=True)
            self.error = e
        finally:
            self._init_done.set()

        if self.error is not None:
            return

        while True:
            self._action_ready.wait()
            self._action_ready.clear()

            if self._stop.is_set():
                return

            try:
                if self._do_reset:
                    obs, info = self.env.reset()
                    self.obs    = obs
                    self.reward = 0.0
                    self.term   = False
                    self.trunc  = False
                    self.info   = info
                else:
                    obs, reward, term, trunc, info = self.env.step(self._action)
                    self.obs    = obs
                    self.reward = reward
                    self.term   = term
                    self.trunc  = trunc
                    self.info   = info
                self.error = None
            except Exception as e:
                self.error = e

            self._result_ready.set()


class VecEnv:
    """Vectorised env over N threads, one env per thread."""

    def __init__(self, cfg: Config, env_class=FRCSimEnv):
        self.num_envs = cfg.num_envs

        # Spawn all workers simultaneously — each constructs its env in its
        # own thread so all NT servers start at the same time.
        self._workers = [
            _EnvWorker(cfg, cfg.nt_port + i, env_class)
            for i in range(self.num_envs)
        ]

        # Wait for all env constructors to finish before returning.
        for w in self._workers:
            w.wait_init()

        self._match_ended = np.zeros(self.num_envs, dtype=bool)

    # ── connection ────────────────────────────────────────────────────────────

    def wait_connected(self, timeout: float = 120.0):
        """Block until every env has at least one NT client connected."""
        deadline = time.monotonic() + timeout
        last_print = 0.0
        while time.monotonic() < deadline:
            counts = [w.connected_count() for w in self._workers]
            if all(c >= 1 for c in counts):
                print(f"  [VecEnv] all {self.num_envs} envs connected")
                return
            now = time.monotonic()
            if now - last_print >= 2.0:
                waiting_ports = [
                    self._workers[i]._port
                    for i, c in enumerate(counts) if c == 0
                ]
                print(f"  [VecEnv] waiting for sims on ports: {waiting_ports}")
                last_print = now
            time.sleep(0.25)
        waiting_ports = [
            self._workers[i]._port
            for i, c in enumerate([w.connected_count() for w in self._workers])
            if c == 0
        ]
        raise RuntimeError(
            f"timed out after {timeout}s - no sim connected on ports: {waiting_ports}"
        )

    # ── reset ─────────────────────────────────────────────────────────────────

    def reset(self) -> np.ndarray:
        self._match_ended[:] = False
        for w in self._workers:
            w.send_reset()
        obs_list = []
        for w in self._workers:
            obs, _, _, _, _ = w.recv()
            obs_list.append(obs)
        return np.stack(obs_list, axis=0)

    # ── step ──────────────────────────────────────────────────────────────────

    def step(self, actions: np.ndarray):
        for i, w in enumerate(self._workers):
            action = np.zeros_like(actions[i]) if self._match_ended[i] else actions[i]
            w.send_step(action)

        obs_list, rewards, terms, truncs, raw_infos = [], [], [], [], []
        for w in self._workers:
            obs, r, term, trunc, info = w.recv()
            obs_list.append(obs)
            rewards.append(r)
            terms.append(term)
            truncs.append(trunc)
            raw_infos.append(info)

        obs     = np.stack(obs_list, axis=0)
        rewards = np.array(rewards, dtype=np.float32)
        term    = np.array(terms,   dtype=bool)
        trunc   = np.array(truncs,  dtype=bool)

        # Merge list-of-dicts → dict-of-arrays (gymnasium VecEnv convention).
        info_keys = set().union(*[d.keys() for d in raw_infos])
        infos: dict = {}
        for k in info_keys:
            vals = [d.get(k, False if k == "match_ended" else 0) for d in raw_infos]
            infos[k] = np.array(vals)

        if "match_ended" in infos:
            self._match_ended |= infos["match_ended"].astype(bool)

        if np.all(self._match_ended):
            obs = self.reset()
            term[:] = True
            trunc[:] = False

        return obs, rewards, term, trunc, infos

    # ── close ─────────────────────────────────────────────────────────────────

    def close(self):
        for w in self._workers:
            w.stop()