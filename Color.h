#ifndef COLOR_H
#define COLOR_H

#include <iostream>
#include <string>

using namespace std;

class Color {
private:
    double r, g, b;
public:
    double getRed() { return r; };
    double getGreen() { return g; };
    double getBlue() { return b; };
    Color();
    Color(double inR, double inG, double inB);
    Color(const Color &rhs);
    Color& operator=(const Color &rhs);
    friend Color operator* (double x, const Color& y);
    friend Color operator* (const Color& x, double y);
    friend Color operator* (const Color& x, const Color& y);
    friend Color operator+ (const Color& x, const Color& y);
    friend ostream& operator<< (ostream &out, Color &cColor);
};

Color::Color() {
    r = 0.0;
    g = 0.0;
    b = 0.0;
}

Color::Color(double inR, double inG, double inB) {
    r = inR;
    g = inG;
    b = inB;
}


Color::Color(const Color &rhs) {
    r = rhs.r;
    g = rhs.g;
    b = rhs.b;
}

Color& Color::operator=(const Color &rhs) {
    if (this == &rhs) {
        return *this;
    }

    r = rhs.r;
    g = rhs.g;
    b = rhs.b;
    return *this;
}

Color operator* (double x, const Color& y) {
    return Color(y.r * x, y.g * x, y.b * x);
}

Color operator* (const Color& x, double y) {
    return y * x;
}

Color operator* (const Color& x, const Color& y) {
    return Color(x.r * y.r, x.g * y.g, x.b * y.b);
}

Color operator+ (const Color& x, const Color& y) {
    return Color(x.r + y.r, x.g + y.g, x.b + y.b);
}

ostream& operator<< (ostream &out, Color &v)
{
    out << "Color "
            << v.r << " "
            << v.g << " "
            << v.b;
    return out;
}

#endif