
vec2 normalize_uv = vec2(0.1591, 0.3183);
vec2 sampleSphericalMap(vec3 v) {
    // U is in the range [-PI, PI], V is [-PI/2, PI/2]
    vec2 uv = vec2(atan(v.z, v.x), asin(v.y));
    // Convert UV to [-0.5, 0.5] in U&V
    uv *= normalize_uv;
    // Convert UV to [0, 1]
    uv += 0.5;
    return uv;
}

vec3 sampleFromInsideSphere(vec2 xi, out float pdf) {
//    Point3f pObj = WarpFunctions::squareToSphereUniform(xi);

//    Intersection it;
//    it.normalGeometric = glm::normalize( transform.invTransT() *pObj );
//    it.point = Point3f(transform.T() * glm::vec4(pObj.x, pObj.y, pObj.z, 1.0f));

//    *pdf = 1.0f / Area();

//    return it;
    return vec3(0.);
}

#if N_AREA_LIGHTS
vec3 DirectSampleAreaLight(int idx,
                           vec3 view_point, vec3 view_nor,
                           int num_lights,
                           out vec3 wiW, out float pdf) {
    AreaLight light = areaLights[idx];
    int type = light.shapeType;
    Ray shadowRay;

    if(type == RECTANGLE) {
        // TODO: Paste your code from hw03 here
        vec2 p = vec2(rng() * 2. - 1., rng() * 2. - 1.);
        vec3 p3 = (light.transform.T * vec4(p, 0., 1.)).xyz;
        vec3 nor = light.transform.invTransT * vec3(0., 0., 1);
        vec3 scale = light.transform.scale;
        float pdfA = 1.f / (2 * 2 * scale.x * scale.y * scale.z);
        vec3 wi = normalize(p3 - view_point);
        float r = length(view_point - p3);
        float absdot = AbsDot(wi, normalize(nor));
        if (absdot < 0.001) {
            pdf = 0.;
            return vec3(0.);
        }
        float pdfw = pdfA * r * r / absdot;
        Ray ray = SpawnRay(view_point, wi);
        Intersection inter = sceneIntersect(ray);
        if (abs(inter.t - length(p3 - ray.origin)) > 0.0001) {
            pdf = 0.;
            return vec3(0.);
        }
        wiW = wi;
        pdf = pdfw;
        return light.Le  * num_lights;
    }
    else if(type == SPHERE) {
        Transform tr = areaLights[idx].transform;

        vec2 xi = vec2(rng(), rng());

        vec3 center = vec3(tr.T * vec4(0., 0., 0., 1.));
        vec3 centerToRef = normalize(center - view_point);
        vec3 tan, bit;

        coordinateSystem(centerToRef, tan, bit);

        vec3 pOrigin;
        if(dot(center - view_point, view_nor) > 0) {
            pOrigin = view_point + view_nor * RayEpsilon;
        }
        else {
            pOrigin = view_point - view_nor * RayEpsilon;
        }

        // Inside the sphere
        if(dot(pOrigin - center, pOrigin - center) <= 1.f) // Radius is 1, so r^2 is also 1
            return sampleFromInsideSphere(xi, pdf);

        float sinThetaMax2 = 1 / dot(view_point - center, view_point - center); // Again, radius is 1
        float cosThetaMax = sqrt(max(0.0f, 1.0f - sinThetaMax2));
        float cosTheta = (1.0f - xi.x) + xi.x * cosThetaMax;
        float sinTheta = sqrt(max(0.f, 1.0f- cosTheta * cosTheta));
        float phi = xi.y * TWO_PI;

        float dc = distance(view_point, center);
        float ds = dc * cosTheta - sqrt(max(0.0f, 1 - dc * dc * sinTheta * sinTheta));

        float cosAlpha = (dc * dc + 1 - ds * ds) / (2 * dc * 1);
        float sinAlpha = sqrt(max(0.0f, 1.0f - cosAlpha * cosAlpha));

        vec3 nObj = sinAlpha * cos(phi) * -tan + sinAlpha * sin(phi) * -bit + cosAlpha * -centerToRef;
        vec3 pObj = vec3(nObj); // Would multiply by radius, but it is always 1 in object space

        shadowRay = SpawnRay(view_point, normalize(vec3(tr.T * vec4(pObj, 1.0f)) - view_point));
        wiW = shadowRay.direction;
        pdf = 1.0f / (TWO_PI * (1 - cosThetaMax));
    }

    Intersection isect = sceneIntersect(shadowRay);
    if(isect.obj_ID == areaLights[idx].ID) {
        // Multiply by N+1 to account for sampling it 1/(N+1) times.
        // +1 because there's also the environment light
        return num_lights * areaLights[idx].Le;
    }
}
#endif

#if N_POINT_LIGHTS
vec3 DirectSamplePointLight(int idx,
                            vec3 view_point, int num_lights,
                            out vec3 wiW, out float pdf) {
    PointLight light = pointLights[idx];
    // TODO: Paste your code from hw03 here
    vec3 p3 = (light.transform.T * vec4(light.pos, 1.)).xyz;
    vec3 wi = normalize(p3 - view_point);
    pdf = 1.;
    wiW = wi;
    Ray ray = SpawnRay(view_point, wi);
    Intersection inter = sceneIntersect(ray);
    float r = length(view_point - p3);
    if (inter.t < length(p3 - ray.origin)) {
        pdf = 0.;
        return vec3(0.);
    }
    return light.Le / r / r * num_lights;
}
#endif

#if N_SPOT_LIGHTS
vec3 DirectSampleSpotLight(int idx,
                           vec3 view_point, int num_lights,
                           out vec3 wiW, out float pdf) {
    SpotLight light = spotLights[idx];
    // TODO: Paste your code from hw03 here
    pdf = 0.;
    vec3 p3 = (light.transform.T * vec4(light.pos, 1.)).xyz;
    vec3 nor = light.transform.invTransT * vec3(0., 0., 1);
    vec3 wi = normalize(p3 - view_point);
    if (dot(nor, -wi) < 0.) return vec3(0.);
    float cosOut = cos(radians(light.outerAngle));
    float cosIn = cos(radians(light.innerAngle));
    if (AbsDot(-wi, nor) < cosOut) return vec3(0.);
    float factor = 1.;
    if (AbsDot(-wi, nor) < cosIn) factor = smoothstep(cosOut, cosIn, AbsDot(-wi, nor));
    pdf = 1.;
    wiW = wi;
    Ray ray = SpawnRay(view_point, wi);
    Intersection inter = sceneIntersect(ray);
    float r = length(view_point - p3);
    if (inter.t < length(p3 - ray.origin)) {
        pdf = 0.;
        return vec3(0.);
    }
    return light.Le * vec3(factor) / r / r * num_lights;
}
#endif

vec3 Sample_Li(vec3 view_point, vec3 nor,
                       out vec3 wiW, out float pdf,
                       out int chosenLightIdx,
                       out int chosenLightID) {
    // Choose a random light from among all of the
    // light sources in the scene, including the environment light
    int num_lights = N_LIGHTS;
#define ENV_MAP 1
#if ENV_MAP
    num_lights +=  1;
#endif
    int randomLightIdx = int(rng() * num_lights);
    chosenLightIdx = randomLightIdx;
    // Chose an area light
    if(randomLightIdx < N_AREA_LIGHTS) {
#if N_AREA_LIGHTS
        chosenLightID = areaLights[chosenLightIdx].ID;
        return DirectSampleAreaLight(randomLightIdx, view_point, nor, num_lights, wiW, pdf);
#endif
    }
    // Chose a point light
    else if(randomLightIdx < N_AREA_LIGHTS + N_POINT_LIGHTS) {
#if N_POINT_LIGHTS
        chosenLightID = pointLights[randomLightIdx - N_AREA_LIGHTS].ID;
        return DirectSamplePointLight(randomLightIdx - N_AREA_LIGHTS, view_point, num_lights, wiW, pdf);
#endif
    }
    // Chose a spot light
    else if(randomLightIdx < N_AREA_LIGHTS + N_POINT_LIGHTS + N_SPOT_LIGHTS) {
#if N_SPOT_LIGHTS
        chosenLightID = spotLights[randomLightIdx - N_AREA_LIGHTS - N_POINT_LIGHTS].ID;
        return DirectSampleSpotLight(randomLightIdx - N_AREA_LIGHTS - N_POINT_LIGHTS, view_point, num_lights, wiW, pdf);
#endif
    }
    // Chose the environment light
    else{
        chosenLightID = -1;
        vec3 wi = squareToHemisphereCosine(vec2(rng(), rng()));
        pdf = squareToHemisphereCosinePDF(wi);
        wiW = LocalToWorld(nor) * wi;
        Ray ray = SpawnRay(view_point, normalize(wiW));
        Intersection inter = sceneIntersect(ray);
        if (inter.t < INFINITY) return vec3(0.);
        vec2 uv = sampleSphericalMap(wiW);
        return texture(u_EnvironmentMap, uv).rgb;
    }
    return vec3(0.);
}

float UniformConePdf(float cosThetaMax) {
    return 1 / (2 * PI * (1 - cosThetaMax));
}

float SpherePdf(Intersection ref, vec3 p, vec3 wi,
                Transform transform, float radius) {
    vec3 nor = ref.nor;
    vec3 pCenter = (transform.T * vec4(0, 0, 0, 1)).xyz;
    // Return uniform PDF if point is inside sphere
    vec3 pOrigin = p + nor * 0.0001;
    // If inside the sphere
    if(DistanceSquared(pOrigin, pCenter) <= radius * radius) {
//        return Shape::Pdf(ref, wi);
        // To be provided later
        return 0.f;
    }

    // Compute general sphere PDF
    float sinThetaMax2 = radius * radius / DistanceSquared(p, pCenter);
    float cosThetaMax = sqrt(max(0.f, 1.f - sinThetaMax2));
    return UniformConePdf(cosThetaMax);
}


float Pdf_Li(vec3 view_point, vec3 nor, vec3 wiW, int chosenLightIdx) {

    Ray ray = SpawnRay(view_point, wiW);

    // Area light
    if(chosenLightIdx < N_AREA_LIGHTS) {
#if N_AREA_LIGHTS
        Intersection isect = areaLightIntersect(areaLights[chosenLightIdx],
                                                ray);
        if(isect.t == INFINITY) {
            return 0.;
        }
        vec3 light_point = ray.origin + isect.t * wiW;
        // If doesn't intersect, 0 PDF
        if(isect.t == INFINITY) {
            return 0.;
        }

        int type = areaLights[chosenLightIdx].shapeType;
        if(type == RECTANGLE) {
            AreaLight light = areaLights[chosenLightIdx];
            vec3 scale = light.transform.scale;
            float pdfA = 1.f / (2 * 2 * scale.x * scale.y * scale.z);
            float r = length(view_point - light_point);
            vec3 nor = light.transform.invTransT * vec3(0., 0., 1);
            vec3 wi = normalize(light_point - view_point);
            float absdot = AbsDot(-wi, nor);
            if (absdot == 0.) return 0.;
            return pdfA * r * r / absdot;
        }
        else if(type == SPHERE) {
            return SpherePdf(isect, light_point, wiW,
                                  areaLights[chosenLightIdx].transform,
                                  1.f);
        }
#endif
    }
    // Point light or spot light
    else if(chosenLightIdx < N_AREA_LIGHTS + N_POINT_LIGHTS ||
            chosenLightIdx < N_AREA_LIGHTS + N_POINT_LIGHTS + N_SPOT_LIGHTS) {
        return 0;
    }
    // Env map
    else {
        vec3 wi = WorldToLocal(nor) * wiW;
        return squareToHemisphereCosinePDF(wi);
    }
}

float PowerHeuristic(int nf, float fPdf, int ng, float gPdf) {
    return pow(nf * fPdf, 2) / (pow(nf * fPdf, 2) + pow(ng * gPdf, 2));
}

vec3 Li_DirectMISInter(Intersection inter, vec3 woW, vec3 view_point) {
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

    // if hit a light source or hit nothing: end
    if (inter.t == INFINITY) {
        return vec3(0.);
    }
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
    // optimized if the chosen light is a spotlight or a pointlight
    if (lightIndex >= N_AREA_LIGHTS && lightIndex < N_AREA_LIGHTS + N_POINT_LIGHTS + N_SPOT_LIGHTS) {
        return color;
    }
    // BSDF part
   Ray rayB = SpawnRay(view_point, wiWB);
   Intersection inter2 = sceneIntersect(rayB);
   if (pdfbb > 0. && inter2.obj_ID == lightID) {
        pdfbl = Pdf_Li(view_point, inter.nor, wiWB, lightIndex);
        wB = PowerHeuristic(1, pdfbb, 1, pdfbl);
        float absdot2 = AbsDot(wiWB, normalize(inter.nor));
        color += wB * fB * inter2.Le * absdot2 / pdfbb;
   }
   else if (pdfbb > 0. && inter2.t == INFINITY && lightID == -1) {
        pdfbl = Pdf_Li(view_point, inter.nor, wiWB, lightIndex);
        wB = PowerHeuristic(1, pdfbb, 1, pdfbl);
        float absdot2 = AbsDot(wiWB, normalize(inter.nor));
        vec2 uv = sampleSphericalMap(rayB.direction);
        color += wB * fB * texture(u_EnvironmentMap, uv).rgb * absdot2 / pdfbb;
   }
   return color;
}
