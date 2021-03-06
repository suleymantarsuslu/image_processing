// ------------------------------
// Written by Suleyman TARSUSLU
// ------------------------------
#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <string>
#include <vector>

#include "types.hpp"
#include "util.hpp"
#include "keypoint.hpp"
#include "descriptor.hpp"

using std::vector;

namespace ceng391 {

class Image {
public:
        Image(int width, int height, int n_channels, int step = -1);
        ~Image();

        static Image* new_gray(int width, int height);
        static Image* new_rgb(int width, int height);
        static Image* new_copy(Image *img);

        int w   () const { return m_width; }
        int h   () const { return m_height; }
        int n_ch() const { return m_n_channels; }
        int step() const { return m_step; }

        uchar*       data()       { return m_data; }
        const uchar* data() const { return m_data; }
        uchar*       data(int y)       { return m_data + y*m_step; }
        const uchar* data(int y) const { return m_data + y*m_step; }

        void set_rect(int x, int y, int width, int height, uchar value);
        void set_rect(int x, int y, int width, int height, uchar red, uchar green, uchar blue);
        void set(uchar value) { set_rect(0, 0, m_width, m_height, value); }
        void set_zero() { set(0); }

        void to_grayscale();
        void to_rgb();

        void rotate(Image *rotated, double theta, double tx, double ty) const;
        void rotate_centered(Image *rotated, double theta) const;

        void smooth_x(float sigma);
        void smooth_y(float sigma);
        void smooth(float sigma_x, float sigma_y);

        short *deriv_x() const;
        short *deriv_y() const;

        bool write_pnm(const std::string& filename) const;
        bool read_pnm (const std::string& filename);

        std::vector<Keypoint> harris_corners(float threshold, float k,
                                             float sigma);

        std::vector<Descriptor> compute_brief(const std::vector<Keypoint>& keys);

        static std::vector<Match> match_brief(const std::vector<Descriptor>& d0,
                                              const std::vector<Descriptor>& d1);
private:
        int m_width;
        int m_height;
        int m_n_channels;
        int m_step;
        uchar* m_data;
};

}

#endif
