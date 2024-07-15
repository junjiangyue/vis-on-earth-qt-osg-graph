#version 130

#define SkipAlpha (.95f)
#define PI (3.14159f)

uniform sampler3D volTex0;
uniform sampler3D volTex1;
uniform sampler2D tfTexPreInt0;
uniform sampler2D tfTexPreInt1;
uniform sampler1D tfTex0;
uniform sampler1D tfTex1;
uniform vec3 eyePos;
uniform vec3 dSamplePos0;
uniform vec3 dSamplePos1;
uniform mat3 rotMat;
uniform float sliceCntrX;
uniform float sliceCntrY;
uniform float sliceCntrZ;
uniform float sliceDirX;
uniform float sliceDirY;
uniform float sliceDirZ;
uniform float lightPosX;
uniform float lightPosY;
uniform float lightPosZ;
uniform float step;
uniform float latitudeMin;
uniform float latitudeMax;
uniform float longtitudeMin;
uniform float longtitudeMax;
uniform float heightMin;
uniform float heightMax;
uniform float ka;
uniform float kd;
uniform float ks;
uniform float shininess;
uniform int maxStepCnt;
uniform bool useSlicing;
uniform bool useShading;
uniform bool useTFPreInt;
uniform bool useMultiVols;

varying vec3 vertex;

struct Hit {
    int isHit;
    float tEntry;
    float tExit;
};
/*
 * ����: intersectSphere
 * ����: �������������ཻ��λ��
 * ����:
 * -- d: �ӵ�����ķ���
 * -- r: ��뾶
 */
Hit intersectSphere(vec3 d, float r) {
    Hit hit = Hit(0, 0.f, 0.f);

    float tVert = -dot(eyePos, d);
    vec3 pVert = eyePos + tVert * d;

    float r2 = r * r;
    float pVert2 = pVert.x * pVert.x + pVert.y * pVert.y + pVert.z * pVert.z;
    if (pVert2 >= r2)
        return hit;
    float l = sqrt(r2 - pVert2);

    hit.isHit = 1;
    hit.tEntry = tVert - l;
    hit.tExit = tVert + l;
    return hit;
}

struct SliceOnSphere {
    vec3 cntr;
    vec3 dir;
};
/*
 * ����: computeSliceOnSphere
 * ����: ���ص���ռ��е�����
 */
SliceOnSphere computeSliceOnSphere() {
    SliceOnSphere ret;

    float lon = longtitudeMin + sliceCntrX * (longtitudeMax - longtitudeMin);
    float lat = latitudeMin + sliceCntrY * (latitudeMax - latitudeMin);
    float h = heightMin + sliceCntrZ * (heightMax - heightMin);
    ret.cntr.z = h * sin(lat);
    h = h * cos(lat);
    ret.cntr.y = h * sin(lon);
    ret.cntr.x = h * cos(lon);

    ret.dir = rotMat * vec3(sliceDirX, sliceDirY, sliceDirZ);

    return ret;
}

vec3 computeShading(vec3 tfCol, vec3 d, vec3 pos, vec3 samplePos, vec3 dSamplePos,
                    sampler3D volTex) {
    vec3 N;
    N.x = texture(volTex, samplePos + vec3(dSamplePos.x, 0, 0)).r -
          texture(volTex, samplePos - vec3(dSamplePos.x, 0, 0)).r;
    N.y = texture(volTex, samplePos + vec3(0, dSamplePos.y, 0)).r -
          texture(volTex, samplePos - vec3(0, dSamplePos.y, 0)).r;
    N.z = texture(volTex, samplePos + vec3(0, 0, dSamplePos.z)).r -
          texture(volTex, samplePos - vec3(0, 0, dSamplePos.z)).r;
    N = rotMat * normalize(N);
    if (dot(N, d) > 0)
        N = -N;

    vec3 p2l = normalize(vec3(lightPosX, lightPosY, lightPosZ) - pos);
    vec3 hfDir = normalize(-d + p2l);

    float ambient = ka;
    float diffuse = kd * max(0, dot(N, p2l));
    float specular = ks * pow(max(0, dot(N, hfDir)), shininess);

    return (ambient + diffuse + specular) * tfCol;
}

/*
 * ����: intersectSlice
 * ����: ���������������ཻ��λ��
 * ����:
 * -- e2pDir: �ӵ�ָ��p�ķ���
 * -- slice: ����ռ��е�����
 */
Hit intersectSlice(vec3 e2pDir, SliceOnSphere slice) {
    Hit hit = Hit(0, 0.f, 0.f);

    float dirstepN = dot(e2pDir, slice.dir);
    if (dirstepN >= 0.f)
        return hit;

    hit.isHit = 1;
    hit.tEntry = dot(slice.dir, slice.cntr) - dot(slice.dir, eyePos);
    hit.tEntry /= dirstepN;
    return hit;
}

void main() {
    vec3 d = normalize(vertex - eyePos);
    Hit hit = intersectSphere(d, heightMax);
    if (hit.isHit == 0)
        discard;
    float tEntry = hit.tEntry;
    vec3 outerX = eyePos + tEntry * d;

    vec3 pos = outerX;
    float r = sqrt(pos.x * pos.x + pos.y * pos.y);
    float lat = atan(pos.z / r);
    r = length(pos);
    float lon = atan(pos.y, pos.x);
    // �ж������������һ�����㣨���������λ�ã���������
    int entryOutOfRng = 0;
    if (lat < latitudeMin)
        entryOutOfRng |= 1;
    if (lat > latitudeMax)
        entryOutOfRng |= 2;
    if (lon < longtitudeMin)
        entryOutOfRng |= 4;
    if (lon > longtitudeMax)
        entryOutOfRng |= 8;
    float tExit = hit.tExit;
    hit = intersectSphere(d, heightMin);
    if (hit.isHit != 0)
        tExit = hit.tEntry;
    // �ж������뿪���λ����������
    pos = eyePos + tExit * d;
    r = sqrt(pos.x * pos.x + pos.y * pos.y);
    lat = atan(pos.z / r);
    lon = atan(pos.y, pos.x);
    // ������λ�þ����ڷ�Χ�ڣ�������������ͬ������Ҫ���������
    gl_FragColor = vec4(0.f, 0.f, 0.f, 1.f);
    if ((entryOutOfRng & 1) != 0 && lat < latitudeMin)
        discard;
    if ((entryOutOfRng & 2) != 0 && lat > latitudeMax)
        discard;
    if ((entryOutOfRng & 4) != 0 && lon < longtitudeMin)
        discard;
    if ((entryOutOfRng & 8) != 0 && lon > longtitudeMax)
        discard;

    float hDlt = heightMax - heightMin;
    float latDlt = latitudeMax - latitudeMin;
    float lonDlt = longtitudeMax - longtitudeMin;
    // ��������
    SliceOnSphere slice;
    if (useSlicing) {
        slice = computeSliceOnSphere();
        hit = intersectSlice(d, slice);
        if (hit.isHit != 0) {
            vec3 pos = eyePos + hit.tEntry * d;
            float r = sqrt(pos.x * pos.x + pos.y * pos.y);
            float lat = atan(pos.z / r);
            r = length(pos);
            float lon = atan(pos.y, pos.x);

            if (lat < latitudeMin || lat > latitudeMax || lon < longtitudeMin ||
                lon > longtitudeMax || r < heightMin || r > heightMax) {
            } else {
                r = (r - heightMin) / hDlt;
                lat = (lat - latitudeMin) / latDlt;
                lon = (lon - longtitudeMin) / lonDlt;

                float scalar = texture(volTex0, vec3(lon, lat, r)).r;
                gl_FragColor = texture(tfTex0, scalar);
                gl_FragColor.a = 1.f;
                return;
            }
        }
    }
    // ִ�й��ߴ����㷨
    vec4 color = vec4(0, 0, 0, 0);
    float tAcc = 0.f;
    float prevScalar0 = -1.f;
    float prevScalar1 = -1.f;
    int stepCnt = 0;
    pos = outerX;
    tExit -= tEntry;
    do {
        r = sqrt(pos.x * pos.x + pos.y * pos.y);
        lat = atan(pos.z / r);
        r = length(pos);
        lon = atan(pos.y, pos.x);

        if (lat < latitudeMin || lat > latitudeMax || lon < longtitudeMin || lon > longtitudeMax) {
        } else if (useSlicing && dot(pos - slice.cntr, slice.dir) >= 0) {
        } else {
            r = (r - heightMin) / hDlt;
            lat = (lat - latitudeMin) / latDlt;
            lon = (lon - longtitudeMin) / lonDlt;

            vec4 tfCol;
            vec3 samplePos = vec3(lon, lat, r);
            float scalar = texture(volTex0, samplePos).r;
            if (prevScalar0 < 0.f)
                prevScalar0 = scalar;
            if (useTFPreInt)
                tfCol = texture(tfTexPreInt0, vec2(prevScalar0, scalar));
            else
                tfCol = texture(tfTex0, scalar);
            prevScalar0 = scalar;

            if (useShading && tfCol.a > 0.f)
                tfCol.rgb = computeShading(tfCol.rgb, d, pos, samplePos, dSamplePos0, volTex0);

            if (useMultiVols) {
                vec4 tfCol1;
                scalar = texture(volTex1, samplePos).r;
                if (prevScalar1 < 0.f)
                    prevScalar1 = scalar;
                if (useTFPreInt)
                    tfCol1 = texture(tfTexPreInt1, vec2(prevScalar1, scalar));
                else
                    tfCol1 = texture(tfTex1, scalar);
                prevScalar1 = scalar;

                if (useShading && tfCol1.a > 0.f)
                    tfCol1.rgb =
                        computeShading(tfCol1.rgb, d, pos, samplePos, dSamplePos1, volTex1);

                float a = tfCol.a / (tfCol.a + tfCol1.a);
                tfCol.rgb = a * tfCol.rgb + (1.f - a) * tfCol1.rgb;
                tfCol.a = max(tfCol.a, tfCol1.a);
            }

            if (tfCol.a > 0.f) {
                if (useTFPreInt)
                    color.rgb = color.rgb + (1.f - color.a) * tfCol.rgb;
                else
                    color.rgb = color.rgb + (1.f - color.a) * tfCol.a * tfCol.rgb;
                color.a = color.a + (1.f - color.a) * tfCol.a;
                if (color.a > SkipAlpha)
                    break;
            }
        }

        pos += step * d;
        tAcc += step;
        ++stepCnt;
    } while (tAcc < tExit && stepCnt <= maxStepCnt);

    gl_FragColor = color;
}
