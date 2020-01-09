// ------------------------------
// Written by Suleyman TARSUSLU
// ------------------------------
#ifndef UTIL_HPP
#define UTIL_HPP

#include <vector>

#include "types.hpp"
#include "keypoint.hpp"
#include "descriptor.hpp"

namespace ceng391 {

class Image;

Image *short_to_image(const short *ptr, int width, int height);

float *gaussian_kernel(float sigma, int *k);

template <typename T>
void copy_to_buffer(float *buffer, const T *src, int n,
                    int border_size, int stride);

template <typename T>
void copy_from_buffer(T *dest, const float *buffer, int n, int stride);

void convolve_buffer(float *buffer, int n, const float *kernel, int k);


short *vec_mul(int n, const short *v0, const short *v1);
void smooth_short_buffer(int w, int h, short *I, float sigma);
float *harris_corner_score(int w, int h, const short *Ix2, const short *Iy2,
                           const short *IxIy, float k);
Image *make_keypoint_image(Image *img, std::vector<Keypoint> *keys);
Image *make_match_image(const std::vector<Match> &matches,
                        const Image &img0, const std::vector<Keypoint> &keys0,
                        const Image &img1, const std::vector<Keypoint> &keys1,
                        int distance_threshold);
// ---------------------------- Template Definitions ------------------------------
template <typename T>
void copy_to_buffer(float *buffer, const T *src, int n, int border_size,
                    int stride)
{
        for (int i = 0; i < n; ++i)
                buffer[border_size + i] = src[i * stride];

        for (int i = 0; i < border_size; ++i) {
                buffer[i] = buffer[border_size];
                buffer[i + n + border_size] = buffer[border_size + n - 1];
        }
}

template <typename T>
void copy_from_buffer(T *dest, const float *buffer, int n, int stride)
{
        for (int i = 0; i < n; ++i)
                dest[i * stride] = buffer[i];
}

}

#endif
