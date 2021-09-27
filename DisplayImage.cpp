#include <stdio.h>

#include <opencv2/opencv.hpp>
using namespace cv;

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
    ret[1] = r0 * q1 + r1 * q0 - r2 * q3 + r3 * q2;
    ret[2] = r0 * q2 + r1 * q3 + r2 * q0 - r3 * q1;
    ret[3] = r0 * q3 - r1 * q2 + r2 * q1 + r3 * q0;
    return ret;
}

float *new_rotation(float u, float v, float *rotation, float inHeight,
                    float inWidth) {
    // Helpful resource for this function
    // https://github.com/DanielArnett/360-VJ/blob/d50b68d522190c726df44147c5301a7159bf6c86/ShaderMaker.cpp#L678
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
    longitude = atan2(x, z);
    // Convert back to the normalized M_PIxel coordinate
    x = (longitude + M_PI) / (2.0 * M_PI);
    y = (latitude + M_PI / 2.0) / M_PI;
    static float uv[2];
    // Convert to xy source M_PIxel coordinate
    uv[1] = y * inHeight;
    uv[0] = x * inWidth;

    return uv;
}

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

            // once in texture space replace current point with point after
            // rotation
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

    float rotation2[4] = {0.9238795, 0, 0, 0.3826834};  // x 45

    Mat out = assembleCubemap(in, 250, rotation);
    Mat out2 = assembleCubemap(in, 250, rotation2);

    imshow("default Image", out);
    imwrite("no_rotation.png", out);
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
