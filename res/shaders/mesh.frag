//-----------------------------------------------------------------------------
// Mesh rendering shader
//
// Copyright 2016 Aleksey Egorov
// Enhanced with specular highlights and Fresnel rim lighting
//-----------------------------------------------------------------------------
uniform vec3 lightDir0;
uniform vec3 lightDir1;
uniform float lightInt0;
uniform float lightInt1;
uniform float ambient;

varying vec3 fragNormal;
varying vec4 fragColor;
varying vec3 fragPosition;

void main() {
    vec3 normal = normalize(fragNormal);
    vec3 viewDir = normalize(-fragPosition);

    // Ambient component
    vec3 result = fragColor.rgb * ambient;

    // Diffuse and specular from light 0
    float diffuse0 = clamp(dot(lightDir0, normal), 0.0, 1.0);
    vec3 halfDir0 = normalize(lightDir0 + viewDir);
    float specular0 = pow(clamp(dot(normal, halfDir0), 0.0, 1.0), 32.0);
    result += fragColor.rgb * diffuse0 * lightInt0 * (1.0 - ambient);
    result += vec3(0.3) * specular0 * lightInt0 * diffuse0;

    // Diffuse and specular from light 1
    float diffuse1 = clamp(dot(lightDir1, normal), 0.0, 1.0);
    vec3 halfDir1 = normalize(lightDir1 + viewDir);
    float specular1 = pow(clamp(dot(normal, halfDir1), 0.0, 1.0), 32.0);
    result += fragColor.rgb * diffuse1 * lightInt1 * (1.0 - ambient);
    result += vec3(0.3) * specular1 * lightInt1 * diffuse1;

    // Fresnel rim lighting - makes edges visible even on dark objects
    float fresnel = pow(1.0 - clamp(dot(viewDir, normal), 0.0, 1.0), 3.0);
    result += vec3(0.15) * fresnel;

    gl_FragColor = vec4(result, fragColor.a);
}
