#version 330

// Inputs from Raylib's default vertex shader
in vec2 fragTexCoord;
in vec4 fragColor;

// Output pixel color
out vec4 finalColor;

// Physics uniforms
uniform float mu;
uniform float C_sat;

// Camera uniforms
uniform vec2 resolution;
uniform vec2 camTarget;
uniform vec2 camOffset;
uniform float camZoom;

void main()
{
    vec2 screenPos = vec2(gl_FragCoord.x, resolution.y - gl_FragCoord.y);

    // Inverse camera transform (Screen to World)
    vec2 worldPos = (screenPos - camOffset) / camZoom + camTarget;

    float x = worldPos.x;
    float y = worldPos.y;

    float r1 = sqrt((x + mu)*(x + mu) + y*y);
    float r2 = sqrt((x - (1.0 - mu))*(x - (1.0 - mu)) + y*y);

    // Prevent division by zero if we are exactly on a planet
    if (r1 < 0.001 || r2 < 0.001) {
        finalColor = vec4(0.0);
        return;
    }

    float C_zvc = x*x + y*y + 2.0*(1.0 - mu)/r1 + 2.0*mu/r2;

		float edge = 0.005 / camZoom;
    float alpha = smoothstep(C_sat + edge, C_sat - edge, C_zvc);

    //if (C_zvc < C_sat) {
    //    finalColor = vec4(0.8, 0.6, 0.1, 0.35); 
    //} else {
    //    finalColor = vec4(0.0);
    //}
		finalColor = vec4(0.8, 0.5, 0.1, alpha * 0.4);
}
