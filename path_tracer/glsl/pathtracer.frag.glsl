
const float FOVY = 19.5f * PI / 180.0;


Ray rayCast() {
    vec2 offset = vec2(rng(), rng());
    vec2 ndc = (vec2(gl_FragCoord.xy) + offset) / vec2(u_ScreenDims);
    ndc = ndc * 2.f - vec2(1.f);

    float aspect = u_ScreenDims.x / u_ScreenDims.y;
    vec3 ref = u_Eye + u_Forward;
    vec3 V = u_Up * tan(FOVY * 0.5);
    vec3 H = u_Right * tan(FOVY * 0.5) * aspect;
    vec3 p = ref + H * ndc.x + V * ndc.y;

    return Ray(u_Eye, normalize(p - u_Eye));
}


// TODO: Implement naive integration
vec3 Li_Naive(Ray ray) {
    vec3 throughput = vec3(1.0f); // accumulated bsdf terms and absdot terms
    Intersection inter;
    vec3 wiW;
    float pdf = 1.;
    int sampledType;
    vec3 f;
    float absdot;
    for (int curStep = 0; curStep <= MAX_DEPTH; curStep++) {
        vec3 woW = -ray.direction;
        inter = sceneIntersect(ray);
        if (length(inter.Le) > 0) {
            return inter.Le * throughput;
        } else if (inter.t == INFINITY) {
            return vec3(0.);
        }
        f = Sample_f(inter, woW, vec2(rng(), rng()), wiW, pdf, sampledType);
        absdot = AbsDot(normalize(wiW), normalize(inter.nor));
        throughput = (pdf != 0.) ? throughput * f * absdot / pdf : vec3(0.);
        ray = SpawnRay(ray.origin + inter.t * ray.direction, wiW);
    }
    return vec3(0.);
}

vec3 Li_Direct(Ray ray) {
    vec3 wiW;
    float pdf = 1.;
    int lightIndex;
    int lightID;
    Intersection inter = sceneIntersect(ray);
    if (length(inter.Le) > 0) {
        return inter.Le;
    } else if (inter.t == INFINITY) {
        return vec3(0.);
    }
    vec3 woW = -ray.direction;
    vec3 view_point = ray.origin + inter.t * ray.direction;
    vec3 L = Sample_Li(view_point, inter.nor, wiW, pdf, lightIndex, lightID);
    vec3 f = f(inter, woW, wiW);
    if (pdf == 0) return vec3(0.);
    float absdot = AbsDot(wiW, normalize(inter.nor));
    return f * absdot * L / pdf;
}

vec3 Li_DirectMIS(Ray ray) {
    // make some variables to store output later
    vec3 wiWL;
    vec3 wiWB;
    float pdfll;
    float pdfbb;
    float pdflb;
    float pdfbl;
    float wL;
    float wB;
    vec3 fL = vec3(0.);
    int lightIndex;
    int lightID;
    int sampledType;
    vec3 color = vec3(0.);

    Intersection inter = sceneIntersect(ray);
    // if hit a light source or hit nothing: end
    if (length(inter.Le) > 0) {
        return inter.Le;
    } else if (inter.t == INFINITY) {
        return vec3(0.);
    }
    vec3 woW = -ray.direction;
    vec3 view_point = ray.origin + inter.t * ray.direction;
    vec3 LL = Sample_Li(view_point, inter.nor, wiWL, pdfll, lightIndex, lightID);
    vec3 fB = Sample_f(inter, woW, vec2(rng(), rng()), wiWB, pdfbb, sampledType);
    // Direct lighting part
    if (pdfll > 0. && length(LL) > 0.) {
        fL = f(inter, woW, wiWL);
    }
    if (length(fL) > 0.) {
        pdflb = Pdf(inter, woW, wiWL);
        wL = PowerHeuristic(1, pdfll, 1, pdflb);
        float absdot = AbsDot(wiWL, normalize(inter.nor));
        color += wL * fL * LL * absdot / pdfll;
    }
    // BSDF part
   Ray rayB = SpawnRay(view_point, wiWB);
   Intersection inter2 = sceneIntersect(rayB);
   if (pdfbb > 0. && inter2.obj_ID == lightID) {
        pdfbl = Pdf_Li(view_point, inter2.nor, wiWB, lightIndex);
        wB = PowerHeuristic(1, pdfbb, 1, pdfbl);
        float absdot2 = AbsDot(wiWB, normalize(inter.nor));
        color += wB * fB * inter2.Le * absdot2 / pdfbb;
   }
   return color;
}

vec3 Li_Full(Ray ray) {
    vec3 throughput = vec3(1.f); // accumulated bsdf terms and absdot terms
    vec3 accumColor = vec3(0.f);
    Intersection inter;
    vec3 wiW;
    float pdf = 1.;
    int sampledType;
    vec3 f;
    float absdot;
    bool prevSpec = false;
    for (int curStep = 0; curStep <= MAX_DEPTH; curStep++) {
        vec3 woW = -ray.direction;
        inter = sceneIntersect(ray);
        if (curStep == 0 || prevSpec) {
            if (inter.t < INFINITY) {
                accumColor += inter.Le * throughput;
                if (length(inter.Le) > 0) return accumColor;
            } else {
# if ENV_MAP
                vec2 uv = sampleSphericalMap(ray.direction);
                accumColor += texture(u_EnvironmentMap, uv).rgb;
# endif
                return accumColor;
            }
        }
        vec3 view_point = ray.origin + inter.t * ray.direction;
        if (inter.material.type < 2 || inter.material.type > 4) {
            accumColor += Li_DirectMISInter(inter, woW, view_point) * throughput;
            prevSpec = false;
        } else {
            prevSpec = true;
        }
        f = Sample_f(inter, woW, vec2(rng(), rng()), wiW, pdf, sampledType);
        absdot = AbsDot(normalize(wiW), normalize(inter.nor));
        throughput = (pdf != 0.) ? throughput * f * absdot / pdf : vec3(0.);
        ray = SpawnRay(view_point, wiW);
    }
    return accumColor;
}

void main()
{
    seed = uvec2(u_Iterations, u_Iterations + 1) * uvec2(gl_FragCoord.xy);

    Ray ray = rayCast();

//    vec3 thisIterationColor = Li_Naive(ray);
//    vec3 thisIterationColor = Li_Direct(ray);
//    vec3 thisIterationColor = Li_DirectMIS(ray);
    vec3 thisIterationColor = Li_Full(ray);

    vec3 prevSum = texture(u_AccumImg, gl_FragCoord.xy / u_ScreenDims).rgb;
    vec3 updated = mix(prevSum, thisIterationColor, 1.f / u_Iterations);
    out_Col = vec4(updated, 1.);
}
