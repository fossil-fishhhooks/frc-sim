#version 330

// Input vertex attributes (from vertex shader)
in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Output fragment color
out vec4 finalColor;

#define MAX_LIGHTS          16
#define LIGHT_DIRECTIONAL   0
#define LIGHT_POINT         1
#define SHININESS           32.0  // Increase for smaller/crisper specular thingys
#define SPECULAR_STRENGTH   0.5

struct Light {
    int enabled;
    int type;
    vec3 position;
    vec3 target;
    vec4 color;
};

// Input lighting values
uniform Light lights[MAX_LIGHTS];
uniform vec4 ambient;
uniform vec3 viewPos;

void main()
{
    vec4 texelColor = texture(texture0, fragTexCoord);
    vec3 normal     = normalize(fragNormal);
    vec3 viewDir    = normalize(viewPos - fragPosition);
    vec4 tint       = colDiffuse * fragColor;

    vec3 diffuse  = vec3(0.0);
    vec3 specular = vec3(0.0);

    for (int i = 0; i < MAX_LIGHTS; i++)
    {
        if (lights[i].enabled != 1) continue;

        vec3  lightDir   = vec3(0.0);
        float falloff = 1.0;

        if (lights[i].type == LIGHT_DIRECTIONAL)
        {
            lightDir = normalize(lights[i].position - lights[i].target);
        }
        else if (lights[i].type == LIGHT_POINT)
        {
            vec3  toLight = lights[i].position - fragPosition;
            float dist    = length(toLight);
            lightDir      = toLight / dist;  // normalize without extra sqrt
            falloff   = 1.0 / (1.0 + 0.09 * dist + 0.032 * dist * dist); // standard realistic falloff
        }

        // diffuse lighting
        float NdotL = max(dot(normal, lightDir), 0.0);
        diffuse += lights[i].color.rgb * NdotL * falloff;


        if (NdotL > 0.0) //dont bother doing lighting for surfacs pointing away from lighting
        {    //blinn reflect not true reflect(). its faster
            vec3  halfDir = normalize(lightDir + viewDir);
            float specCo  = pow(max(dot(normal, halfDir), 0.0), SHININESS);
            specular += lights[i].color.rgb * specCo * falloff;
        }
    }

    // total = diffuse + specular + ambient
    vec3 ambientColor = ambient.rgb * tint.rgb;
    vec3 litColor     = texelColor.rgb * tint.rgb * diffuse
                      + texelColor.rgb * specular * SPECULAR_STRENGTH   //
                      + texelColor.rgb * ambientColor;

    finalColor = vec4(litColor, texelColor.a * tint.a);

    // display correction (genuinely no idea why this is a thing)
    finalColor = pow(finalColor, vec4(1.0 / 2.2));
}
