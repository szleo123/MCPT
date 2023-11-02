#version 330 core

uniform sampler2D u_Texture;
uniform vec2 u_ScreenDims;
uniform int u_Iterations;

in vec3 fs_Pos;
in vec2 fs_UV;

out vec4 out_Col;
void main()
{
    vec4 color = texture(u_Texture, fs_UV);
    // TODO: Apply the Reinhard operator and gamma correction
    // before outputting color.
    const float gamma = 2.2;
    //reinhard
    vec3 mapped = color.rgb / (color.rgb + vec3(1.));
    //gamma correction
    mapped = pow(mapped, vec3(1.0 / gamma));

    out_Col = vec4(mapped, 1.);
}
