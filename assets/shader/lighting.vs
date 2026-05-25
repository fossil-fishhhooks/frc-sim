#version 330

in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec4 vertexColor;
in vec3 vertexNormal;

uniform mat4 mvp;
uniform mat4 matModel;

out vec3 fragPosition;
out vec2 fragTexCoord;
out vec4 fragColor;
out vec3 fragNormal;

void main()
{
    fragPosition = vec3(matModel * vec4(vertexPosition, 1.0));
    fragTexCoord = vertexTexCoord;
    fragColor    = vertexColor;
    // Normal matrix: inverse-transpose of model (handles non-uniform scale)
    fragNormal   = normalize(mat3(transpose(inverse(matModel))) * vertexNormal);

    gl_Position  = mvp * vec4(vertexPosition, 1.0);
}
