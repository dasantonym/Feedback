#version 120
#extension GL_ARB_texture_rectangle : enable

// ping pong inputs
uniform sampler2DRect particles0;
uniform sampler2DRect particles1;

uniform vec3 momentCenter;
uniform vec2 viewSize;
uniform float radiusSquared;
uniform float elapsed;

void main()
{
    vec3 pos = texture2DRect(particles0, gl_TexCoord[0].st).xyz;
    vec3 vel = texture2DRect(particles1, gl_TexCoord[0].st).xyz;

    // mouse attraction
    vec3 direction = momentCenter - pos.xyz;
    float distSquared = dot(direction, direction);
    float magnitude = viewSize.y * (1.0 - distSquared / radiusSquared);
    vec3 force = step(distSquared, radiusSquared) * magnitude * normalize(direction);

    // gravity
    force += vec3(0.0, 10.0, 0.0);

    // accelerate
    vel += elapsed * force;

    // bounce off the sides
    vel.x *= step(abs(pos.x), viewSize.x * 0.5) * 2.0 - 1.0;
    vel.y *= step(abs(pos.y), viewSize.y * 0.5) * 2.0 - 1.0;
    
    // damping
    vel *= 0.995;
    
    // move
    pos += elapsed * vel;
    
    gl_FragData[0] = vec4(pos, 1.0);
    gl_FragData[1] = vec4(vel, 0.0);
}