
vec3 squareToDiskConcentric(vec2 xi) {
    float ang, r;
    float a = 2 * xi.x - 1;
    float b = 2 * xi.y - 1;
    if (a == 0 && b == 0) {
        r = 0;
        ang = 0;
    } else if (a*a > b*b) {
        r = a;
        ang = (PI / 4) * (b/a);
    } else {
        r = b;
        ang = (PI / 2) - (PI / 4) * (a/b);
    }
    return vec3(r * cos(ang), r * sin(ang), 0.);
}

vec3 squareToHemisphereCosine(vec2 xi) {
    float ang, r, x, y, z;
    float a = 2 * xi.x - 1;
    float b = 2 * xi.y - 1;
    if (a == 0 && b == 0) {
        r = 0;
        ang = 0;
    } else if (a*a > b*b) {
        r = a;
        ang = (PI / 4) * (b/a);
    } else {
        r = b;
        ang = (PI / 2) - (PI / 4) * (a/b);
    }
    x = r * cos(ang);
    y = r * sin(ang);
    z = sqrt(max(0, 1 - x*x - y*y));
    return vec3(x, y, z);
}

float squareToHemisphereCosinePDF(vec3 sample) {
    return CosTheta(sample) / PI;
}

vec3 squareToSphereUniform(vec2 sample) {
    float x, y, z;
    z = 1 - 2 * sample.x;
    x = cos(2 * PI * sample.y) * sqrt(1 - z*z);
    y = sin(2 * PI * sample.y) * sqrt(1 - z*z);
    return vec3(x, y, z);
}

float squareToSphereUniformPDF(vec3 sample) {
    return INV_FOUR_PI;
}
