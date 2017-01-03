#version 120
#extension GL_ARB_texture_rectangle : enable

uniform sampler2DRect particles0;
uniform sampler2DRect particles1;

uniform vec3 momentCenter;
uniform float ageSeconds;
uniform float energy;

void main()
{
    vec3 pos = texture2DRect(particles0, gl_TexCoord[0].st).xyz;
    vec3 vel = texture2DRect(particles1, gl_TexCoord[0].st).xyz;

    float amod = smoothstep(0.0, 1.0, energy) * 2.0 - 1.0;
    gl_FragColor = vec4(1.0, 0.4 + amod * 0.4 , 0.1, 0.03);
}
