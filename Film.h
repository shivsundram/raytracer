#ifndef FILM_H
#define FILM_H

#include "lodepng.h"

#include <vector>
#include <iostream>

using namespace std;


//CODE FOR WRITING TO AN IMAGE FILE (LODEPNG LIBRARY),
//CITATION: code relating to the writeimage and commit methods is ADAPTED/BORROWED/COPIED FROM THE EXAMPLE FILE ON THE LODEPNG WEBSITE.
//I DO NOT ASSUME 100% CREDIT FOR THAT CODE. ALL DUE CREDIT IS GIVEN TO THE MAKERS OF THE LIBRARY.

//"g++ lodepng.cpp example_encode.cpp -ansi -pedantic -Wall -Wextra -O3"

//From lodepng example: "Encode from raw pixels to disk with a single function call"
//"The image argument has width * height RGBA pixels or width * height * 4 bytes"

//CITATION: code for constructing, adding, multiplying, transforming etc. vectors is powered by the Eigen library. All due credits is given to the makers
//of this library

//ALL OTHER CODE WRITTEN BY SHIV SUNDRAM AND ANDREY ELENSKIY


class Bucket{
public:
    int n; 
    Color c; 
    Bucket();
    void add(double r, double g, double b);

};

Bucket::Bucket(){
    Color initial(1.0f, 0.0f, 0.0f);
    c = initial; 
    n=0;
}

void Bucket::add(double r, double g, double b){
   // double red=(c.getRed()*n+r)/( (double) (n+1));
    //double green=(c.getGreen()*n+g)/( (double) (n+1));
    //double blue=(c.getBlue()*n+b)/;
    double red=(c.getRed()+r);
    double green=(c.getGreen()+g);
    double blue=(c.getBlue()+b);
    c=Color(red, green, blue);
    n=n+1; 
}

class Film {

private:
    int width, height;
    vector<Bucket> buckets; 
    vector<unsigned char> image;
public:
    Film(int widthIn, int heightIn);
    void writeImage();
    void commit(int x, int y, double r, double g, double b);
    void convert(); 
};

void Film::commit(int x, int y, double r, double g, double b){
    buckets[width * y + x].add(r,g,b);
}

Film::Film(int widthIn, int heightIn){
    width = widthIn;
    height = heightIn;
    image.resize(4*width * height );
    buckets.resize(width * height );
    for (int y=0; y<height; y++){
        for (int x=0; x<width; x++){
            buckets.push_back(Bucket());
        }
    }
}

void Film::writeImage() {
    //NOTE: this sample will overwrite the file or rayprint.png without warning!
    const char* filename = "rayprint.png";
    convert();
    //Encode the image
    unsigned error = lodepng::encode(filename, image, width, height);
    //if there's an error, display it
    if (error) cout << "encoder error " << error << ": " << lodepng_error_text(error) << endl;
}


//saves image to filename given as argument. Warning, this overwrites the file without warning!. at the moment, only one sample per picture
void Film::convert() {
    for (int y=0; y<height; y++){
        for (int x=0; x<width; x++){
            double count = buckets[width * y + x].n;
            image[4 * width * y + 4 * x + 0] = buckets[width * y + x].c.getRed() / count; //RED
            image[4 * width * y + 4 * x + 1] = buckets[width * y + x].c.getGreen() / count ;
            image[4 * width * y + 4 * x + 2] = buckets[width * y + x].c.getBlue() / count;
            image[4 * width * y + 4 * x + 3] = 255; //ALPHA for controlling transaperncy: MAINTAIN AT 255   
        }
    }
}

#endif
