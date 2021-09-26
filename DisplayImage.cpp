#include <stdio.h>

#include <opencv2/opencv.hpp>
using namespace cv;

float *uvToXYZ(float u, float v) {
    float lat = v * M_PI - M_PI / 2.0;
    float lon = u * 2.0 * M_PI - M_PI;
    printf("lat: %f, lon: %f\n", lat, lon);

    static float point[3];
    point[0] = cos(lat) * sin(lon);
    point[1] = sin(lat);
    point[2] = cos(lat) * cos(lon);

    return point;
}
float *xyzToUV(float x, float y, float z) {
    // printf("Y:%f \n", y);
    float lat = asin(y);
    // printf("lat: %f \n", lat);

    float lon = atan2(x, z);
    // printf("X: %f, Y:%f, Z:%f\n", x, y, z);

    printf("lat: %f, lon: %f\n", lat, lon);
    static float uv[2];
    uv[0] = (lon + M_PI) / (2.0 * M_PI);
    uv[1] = (lat + M_PI / 2.0) / M_PI;
    // uv[0] = atan2(x, z);
    // uv[1] = asin(y);
    return uv;
}
static float *quaternion_mult(const float q[4], const float r[4]) {
    static float ret[4];
    // printf("before1 w: %f, X: %f, Y: %f, Z: %f\n", q[0], q[1], q[2], q[3]);
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];
    float r0 = r[0];
    float r1 = r[1];
    float r2 = r[2];
    float r3 = r[3];
    ret[0] = r0 * q0 - r1 * q1 - r2 * q2 - r3 * q3;
    // printf("before2 w: %f, X: %f, Y: %f, Z: %f\n", q[0], q[1], q[2], q[3]);

    ret[1] = r0 * q1 + r1 * q0 - r2 * q3 + r3 * q2;
    // printf("before3 w: %f, X: %f, Y: %f, Z: %f\n", q[0], q[1], q[2], q[3]);

    ret[2] = r0 * q2 + r1 * q3 + r2 * q0 - r3 * q1;
    // printf("before4 w: %f, X: %f, Y: %f, Z: %f\n", q[0], q[1], q[2], q[3]);

    // printf("before2 w: %f, X: %f, Y: %f, Z: %f\n", r[0], r[1], r[2], r[3]);
    ret[3] = r0 * q3 - r1 * q2 + r2 * q1 + r3 * q0;

    // printf("w: %f, X: %f, Y: %f, Z: %f\n", ret[0], ret[1], ret[2], ret[3]);

    return ret;
}
float *uvRotation(float u, float v, float *rotation) {
    float *xyz;
    printf("\nU:%f, V:%f\n", u, v);
    xyz = uvToXYZ(u, v);
    printf("X: %f, Y:%f, Z:%f\n", xyz[0], xyz[1], xyz[2]);
    float p[4];
    p[0] = 0;
    p[1] = xyz[0];
    p[2] = xyz[1];
    p[3] = xyz[2];
    // float rotationInv[4] = {rotation[0], -rotation[1], -rotation[2],
    //                         -rotation[3]};
    // float *p_ret = quaternion_mult((float *)p, (float *)rotation);
    // p_ret = quaternion_mult((float *)p_ret, (float *)rotationInv);
    float *p_ret = p;
    // printf("AFTER ROTATION\n");
    printf("X: %f, Y:%f, Z:%f\n", p_ret[1], p_ret[2], p_ret[3]);

    static float *uv;
    uv = xyzToUV(p_ret[1], p_ret[2], p_ret[3]);
    printf("U:%f, V:%f\n", uv[0], uv[1]);

    // printf("U:%f, V:%f\n", uv[0], uv[1]);

    return uv;
}
float *uvw_rotation(float u, float v, float *rotation) {
    float p[4];
    p[0] = 0;
    p[1] = u;
    p[2] = v;
    p[3] = 1;
    float rotationInv[4] = {rotation[0], -rotation[1], -rotation[2],
                            -rotation[3]};
    float *p_ret = quaternion_mult((float *)p, (float *)rotation);
    p_ret = quaternion_mult((float *)p_ret, (float *)rotationInv);
    // printf("X: %f, Y:%f, Z:%f\n", p_ret[1], p_ret[2], p_ret[3]);

    static float uv[2];
    uv[0] = p_ret[1];
    uv[1] = p_ret[2];
    return uv;
}

float *new_rotation(float u, float v, float *rotation, float inHeight,
                    float inWidth) {
    u = u / inWidth;
    v = v / inHeight;
    float latitude = v * M_PI - M_PI / 2.0;
    float longitude = u * 2.0 * M_PI - M_PI;
    // Create a ray from the latitude and longitude
    float p[4];
    p[0] = 0;
    p[1] = cos(latitude) * sin(longitude);
    p[2] = sin(latitude);
    p[3] = cos(latitude) * cos(longitude);

    // Rotate the ray based on the user input
    float rotationInv[4] = {rotation[0], -rotation[1], -rotation[2],
                            -rotation[3]};
    float *p_ret = quaternion_mult(
        quaternion_mult((float *)rotation, (float *)p), (float *)rotationInv);

    float x = p_ret[1];
    float y = p_ret[2];
    float z = p_ret[3];
    // Convert back to latitude and longitude
    latitude = asin(y);
    // Manually implement `longitude = atan2(x, z);`
    if (z > 0.0) {
        longitude = atan(x / z);
    } else if (z < 0.0 && x >= 0.0) {
        longitude = atan(x / z) + M_PI;
    } else if (z < 0.0 && x < 0.0) {
        longitude = atan(x / z) - M_PI;
    } else if (z == 0.0 && x > 0.0) {
        longitude = M_PI / 2.0;
    } else if (z == 0.0 && x < 0.0) {
        longitude = -M_PI / 2.0;
    }
    // Convert back to the normalized M_PIxel coordinate
    x = (longitude + M_PI) / (2.0 * M_PI);
    y = (latitude + M_PI / 2.0) / M_PI;
    static float uv[2];
    // Convert to xy source M_PIxel coordinate
    uv[1] = y * inHeight;
    uv[0] = x * inWidth;
    return uv;
}
// float *pointToLatLon(float x, float y, float z) {
//     x = asin(y);
//     y = atan2(x, z);
//     float *latlon;
//     latlon[0] = x;
//     latlon[1] = y;
//     return latlon;
// }
// float *latLonToUv(float lat, float lon) {
//     float *uv;
//     uv[0] = (lon + M_PI) / (2.0 * M_PI);
//     uv[1] = (lat + M_PI / 2.0) / M_PI;
//     return uv;
// }
// float *point_rotation_by_quaternion(float point[3], float q[4]) {
//     float r[4];
//     r[0] = 0;
//     r[1] = point[1];
//     r[2] = point[2];
//     r[3] = point[3];
//     float q_conj[4];
//     q_conj[0] = q[0];
//     q_conj[1] = -1 * q[1];
//     q_conj[2] = -1 * q[2];
//     q_conj[4] = -1 * q[3];
//     float *result = quaternion_mult(quaternion_mult(q, r), q_conj);
//     float *ret;
//     ret[0] = result[1];
//     ret[1] = result[2];
//     ret[2] = result[3];
//     return ret;
// }
// Convert latitude, longitude to x, y pixel coordinates on an
// equirectangular

// createCubeMapFace implemented by https://stackoverflow.com/a/34720686

// Define our six cube faces.
// 0 - 3 are side faces, clockwise order
// 4 and 5 are top and bottom, respectively
float faceTransform[6][2] = {{0, 0},         {M_PI / 2, 0},  {M_PI, 0},
                             {-M_PI / 2, 0}, {0, -M_PI / 2}, {0, M_PI / 2}};

// Map a part of the equirectangular panorama (in) to a cube face
// (face). The ID of the face is given by faceId. The desired
// width and height are given by width and height.
inline void createCubeMapFace(const Mat &in, Mat &face, float rotation[4],
                              int faceId = 0, const int width = -1,
                              const int height = -1) {
    float inWidth = in.cols;
    float inHeight = in.rows;

    // Allocate map
    Mat mapx(height, width, CV_32F);
    Mat mapy(height, width, CV_32F);

    // Calculate adjacent (ak) and opposite (an) of the
    // triangle that is spanned from the sphere center
    // to our cube face.
    const float an = sin(M_PI / 4);
    const float ak = cos(M_PI / 4);

    float ftu = faceTransform[faceId][0];
    float ftv = faceTransform[faceId][1];
    // printf("\nftu: %f, ftv: %f\n", ftu, ftv);
    // float *uv;
    // uv = uvw_rotation(ftu, ftv, (float *)rotation);
    // ftu = uv[0];
    // ftv = uv[1];
    // printf("ftu: %f, ftv: %f\n", uv[0], uv[1]);

    // For each point in the target image,
    // calculate the corresponding source coordinates.
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Map face pixel coordinates to [-1, 1] on plane
            float nx = ((float)(y) / (float)height - 0.5f);
            float ny = ((float)(x) / (float)width - 0.5f);

            // printf("test");
            nx *= 2;
            ny *= 2;
            // printf("nx: %f, ny %f\n", nx, ny);

            // Map [-1, 1] plane coords to [-an, an]
            // thats the coordinates in respect to a unit sphere
            // that contains our box.
            nx *= an;
            ny *= an;
            // printf("Mapped to unit circle\n");
            // printf("nx: %f, ny %f\n", nx, ny);

            float u, v;

            // Project from plane to sphere surface.
            if (ftv == 0) {
                // Center faces
                u = atan2(nx, ak);
                v = atan2(ny * cos(u), ak);
                u += ftu;
            } else if (ftv > 0) {
                // Bottom face
                float d = sqrt(nx * nx + ny * ny);
                v = M_PI / 2 - atan2(d, ak);
                u = atan2(ny, nx);
            } else {
                // Top face
                float d = sqrt(nx * nx + ny * ny);
                v = -M_PI / 2 + atan2(d, ak);
                u = atan2(-ny, nx);
            }

            // Map from angular coordinates to [-1, 1], respectively.
            // printf("U:%f ", u);
            // printf("V:%f\n", v);

            u = u / (M_PI);
            v = v / (M_PI / 2);

            // Warp around, if our coordinates are out of bounds.
            while (v < -1) {
                v += 2;
                u += 1;
            }
            while (v > 1) {
                v -= 2;
                u += 1;
            }

            while (u < -1) {
                u += 2;
            }
            while (u > 1) {
                u -= 2;
            }

            // Map from [-1, 1] to in texture space
            u = u / 2.0f + 0.5f;
            v = v / 2.0f + 0.5f;
            u = u * (inWidth - 1);
            v = v * (inHeight - 1);
            float *uv;
            uv = new_rotation(u, v, (float *)rotation, inHeight - 1,
                              inWidth - 1);
            u = uv[0];
            v = uv[1];
            // Save the result for this pixel in map

            mapx.at<float>(x, y) = u;
            mapy.at<float>(x, y) = v;
            // exit(1);
        }
    }

    // // Recreate output image if it has wrong size or type.
    // if(face.cols != width || face.rows != height ||
    //     face.type() != in.type()) {
    //     face = Mat(width, height, in.type());
    // }

    // Do actual resampling using OpenCV's remap
    remap(in, face, mapx, mapy, cv::INTER_LINEAR, BORDER_CONSTANT,
          Scalar(0, 0, 0));
}
Mat importImage(String path) {
    Mat image = imread(path, CV_32F);
    if (!image.data) {
        printf("No image data \n");
    }
    return image;
}

Mat assembleCubemap(Mat in, int cubeSize, float rotation[4]) {
    Mat left, front, right, back, top, bottom;

    // Generate each face of the cubemap
    createCubeMapFace(in, front, rotation, 0, cubeSize, cubeSize);
    createCubeMapFace(in, right, rotation, 1, cubeSize, cubeSize);
    createCubeMapFace(in, back, rotation, 2, cubeSize, cubeSize);
    createCubeMapFace(in, left, rotation, 3, cubeSize, cubeSize);
    createCubeMapFace(in, top, rotation, 4, cubeSize, cubeSize);
    createCubeMapFace(in, bottom, rotation, 5, cubeSize, cubeSize);

    Mat img(cubeSize * 3, cubeSize * 4, front.type());
    img.setTo(cv::Scalar(.75, .75, 0));

    // organize different faces into cubemap
    // can probably fix rotation within createCubeMapFace
    cv::rotate(top, top, cv::ROTATE_90_CLOCKWISE);
    cv::rotate(bottom, bottom, cv::ROTATE_90_COUNTERCLOCKWISE);

    top.copyTo(img(Rect(250, 0, top.cols, top.rows)));
    left.copyTo(img(Rect(0, 250, left.cols, left.rows)));
    front.copyTo(img(Rect(250, 250, front.cols, front.rows)));
    right.copyTo(img(Rect(500, 250, right.cols, right.rows)));
    back.copyTo(img(Rect(750, 250, back.cols, back.rows)));
    bottom.copyTo(img(Rect(250, 500, bottom.cols, bottom.rows)));

    return img;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    Mat in = importImage(argv[1]);
    float rotation[4] = {1, 0, 0, 0};
    // float rotation2[4] = {0.9238795, 0, 0.3826834, 0};// y 45

    float rotation2[4] = {0.9238795, 0, 0, 0.3826834};

    Mat out = assembleCubemap(in, 250, rotation);
    Mat out2 = assembleCubemap(in, 250, rotation2);

    // imshow("Input", in);
    imshow("default Image", out);
    // imwrite("no_rotation.png", out);
    imshow("modified Image", out2);

    waitKey(0);

    // U:0.781382 V:0.343616
    // float u = 0.781382;
    // float v = 0.343616;

    // printf("X: %f, Y:%f, Z:%f\n", p_ret[1], p_ret[2], p_ret[3]);
    // const float p[4] = {0, 1, 0, 0};
    // const float rotation[4] = {0.7071203316249954, 0, 0.7071203316249954, 0};
    // const float rotationInv[4] = {0.7071203316249954, -0,
    // -0.7071203316249954,

    // float *p_ret = quaternion_mult(
    //     quaternion_mult((float *)rotation, (float *)p), (float
    //     *)rotationInv);
    // printf("w: %f, X: %f, Y: %f, Z: %f\n", p_ret[0], p_ret[1], p_ret[2],
    //        p_ret[3]);
    // float *p_ret = new float[4];
    // p_ret = quaternion_mult(quaternion_mult(rotation, p), rotationInv);
    // printf("w: %f, X: %f, Y: %f, Z: %f\n", p_ret[0], p_ret[1], p_ret[2],
    //        p_ret[3]);
    // float q[4] = {0, 0.707, 0, -0.707};
    // float r[4] = {0.707, 0, -0.707, 0};
    // printf("%f", r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0]);

    return 0;
}
