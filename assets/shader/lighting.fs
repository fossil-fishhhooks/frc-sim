#version 330

in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;
in vec4 fragPosLightSpace;

uniform sampler2D texture0;     // diffuse (raylib default white if no texture)
uniform sampler2D shadowMap;    // depth texture from light pass
uniform vec4      colDiffuse;

out vec4 finalColor;

#define MAX_LIGHTS        8
#define LIGHT_DIRECTIONAL  0
#define LIGHT_POINT        1

// Tuneables
#define SHININESS         78.0   // higher = tighter specular
#define SPECULAR_STRENGTH  0.6
#define SHADOW_BIAS        0.005 //
#define FRESNEL_STRENGTH   0.35  // rim light intensity
#define FRESNEL_POWER      3.0   // rim falloff sharpness
#define MIN_SHADOW         32

struct Light {
    int   enabled;
    int   type;
    vec3  position;
    vec3  target;
    vec4  color;
};

uniform Light lights[MAX_LIGHTS];
uniform vec4  ambient;
uniform vec3  viewPos;

// ── PCF shadow (3×3 kernel, 9 taps) ─────────────────────────────────────────
// Returns 0.0 = fully shadowed, 1.0 = fully lit
float ShadowPCF(vec4 fragPosLS, vec3 normal, vec3 lightDir)
{
    // Perspective divide → NDC [-1, 1]
    vec3 projCoords = fragPosLS.xyz / fragPosLS.w;

    // Remap to [0, 1] for texture lookup
    projCoords = projCoords * 0.5 + 0.5;

    // Outside the light frustum — treat as lit
    if (projCoords.z > 1.0)
        return 1.0;

    float currentDepth = projCoords.z;

    // Slope-scaled bias: steeper surfaces need more bias to avoid acne
    float bias = max(SHADOW_BIAS * (1.0 - dot(normal, lightDir)), SHADOW_BIAS * 0.1);

    // 3×3 PCF — average 9 shadow map taps offset by one texel each
    vec2 texelSize = 1.0 / vec2(textureSize(shadowMap, 0));
    float shadow = 0.0;
    for (int x = -1; x <= 1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
        {
            float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r;
            shadow += (currentDepth - bias > pcfDepth) ? 0.0 : 1.0;
        }
    }
    return mix(MIN_SHADOW, 1.0, shadow / 9.0);
}

// ── Fresnel rim ──────────────────────────────────────────────────────────────
// Schlick approximation — bright edge when surface is perpendicular to view
float FresnelRim(vec3 normal, vec3 viewDir)
{
    float cosTheta = max(dot(normal, viewDir), 0.0);
    return FRESNEL_STRENGTH * pow(1.0 - cosTheta, FRESNEL_POWER);
}

void main()
{
    vec4 texelColor = texture(texture0, fragTexCoord);
    vec3 normal     = normalize(fragNormal);
    vec3 viewDir    = normalize(viewPos - fragPosition);
    vec4 tint       = colDiffuse * fragColor;

    vec3 diffuse  = vec3(0.0);
    vec3 specular = vec3(0.0);
    float shadowFactor = 1.0;   // accumulated from shadow-casting lights

    for (int i = 0; i < MAX_LIGHTS; i++)
    {
        if (lights[i].enabled != 1) continue;

        vec3  lightDir = vec3(0.0);
        float falloff  = 1.0;

        if (lights[i].type == LIGHT_DIRECTIONAL)
        {
            lightDir = normalize(lights[i].position - lights[i].target);
        }
        else // LIGHT_POINT
        {
            vec3  toLight = lights[i].position - fragPosition;
            float dist    = length(toLight);
            lightDir      = toLight / dist;
            falloff = 1.0 / (1.0 + 0.02 * dist + 0.005 * dist * dist);  // much gentler
        }

        float NdotL = max(dot(normal, lightDir), 0.0);

        // Shadow test for the first enabled light (the overhead bar)
        // Other lights are fill lights — no shadow casting
        if (i == 0)
            shadowFactor = ShadowPCF(fragPosLightSpace, normal, lightDir);

        diffuse += lights[i].color.rgb * NdotL * falloff;

        if (NdotL > 0.0)
        {
            vec3  halfDir = normalize(lightDir + viewDir);
            float specCo  = pow(max(dot(normal, halfDir), 0.0), SHININESS);
            specular += lights[i].color.rgb * specCo * falloff;
        }
    }

    // Fresnel rim — uses ambient light color so it reads as bounce light
    float rim = FresnelRim(normal, viewDir);
    vec3 rimColor = ambient.rgb * rim * 2.0;

    // Combine: shadow attenuates diffuse + specular but not ambient or rim
    // (ambient and rim simulate indirect light that wraps around shadows)
    vec3 ambientColor = ambient.rgb * tint.rgb;
    vec3 litColor     = texelColor.rgb * tint.rgb * diffuse  * shadowFactor
                      + texelColor.rgb * specular * SPECULAR_STRENGTH * shadowFactor
                      + texelColor.rgb * ambientColor
                      + texelColor.rgb * tint.rgb * rimColor;

    finalColor = vec4(litColor, texelColor.a * tint.a);


    finalColor = pow(finalColor, vec4(1.0 / 2.2));
}
