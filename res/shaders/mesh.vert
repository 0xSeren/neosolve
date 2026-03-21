//-----------------------------------------------------------------------------
// Mesh rendering shader
//
// Copyright 2016 Aleksey Egorov
//-----------------------------------------------------------------------------
attribute vec3 pos;
attribute vec3 nor;
attribute vec4 col;

uniform mat4 modelview;
uniform mat4 projection;

varying vec3 fragNormal;
varying vec4 fragColor;
varying vec3 fragPosition;

void main() {
    vec4 viewPos = modelview * vec4(pos, 1.0);
    fragNormal = vec3(modelview * vec4(nor, 0.0));
    fragColor = col;
    fragPosition = viewPos.xyz;

    gl_Position = projection * viewPos;
}
