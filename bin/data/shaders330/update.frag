#version 330

// ping pong inputs
uniform sampler2DRect particles0;
uniform sampler2DRect particles1;

uniform vec3 momentCenter;
uniform vec2 viewSize;
uniform float radiusSquared;
uniform float ageSeconds;
uniform float elapsed;
uniform float energy;

in vec2 texCoordVarying;

layout(location = 0) out vec4 posOut;
layout(location = 1) out vec4 velOut;

void main()
{
    vec3 pos = texture(particles0, texCoordVarying.st).xyz;
    vec3 vel = texture(particles1, texCoordVarying.st).xyz;
    
    // attraction
    vec3 direction = momentCenter - pos.xyz;
    float distSquared = dot(direction, direction);
    float magnitude = viewSize.y * (1.0 - distSquared / radiusSquared);
    vec3 force = step(distSquared, radiusSquared) * magnitude * normalize(direction) * energy;
    
    // gravity
    force += vec3(0.0, 10.0, 0.0);
    
    // acceleration
    vel += elapsed * force;
    
    // bounce off walls
    vel.x *= step(abs(pos.x), viewSize.x * 0.5) * 2.0 - 1.0;
    vel.y *= step(abs(pos.y), viewSize.y * 0.5) * 2.0 - 1.0;
    
    // damping
    vel *= 0.995 - (energy * 2.0 - 1.0) * 0.01;
    
    // move
    pos += elapsed * vel * energy;
    
    posOut = vec4(pos, 1.0);
    velOut = vec4(vel, 0.0);
}