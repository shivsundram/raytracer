#ifndef FILM_H
#define FILM_H

#include "lodepng.h"

#include <vector>
#include <iostream>

#include "Color.h"

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
    void add(Color);

};

Bucket::Bucket(){
    Color initial(0.0, 0.0, 0.0);
    c = initial;
    n=0;
}

void Bucket::add(Color inC){
    c = c + inC;
    n++;
}

class Film {

private:
    int width, height;
    vector<Bucket> buckets;
    vector<unsigned char> image;
public:
    Film(int widthIn, int heightIn);
    void writeImage(const char* name);
    void commit(int x, int y, Color c);
    void convert();
};

void Film::commit(int x, int y, Color c){
    buckets[width * y + x].add(c);
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

void Film::writeImage(const char* name) {
    //NOTE: this sample will overwrite the file or rayprint.png without warning!


    convert();
    //Encode the image
    unsigned error = lodepng::encode(name, image, width, height);
    //if there's an error, display it
    if (error) cout << "encoder error " << error << ": " << lodepng_error_text(error) << endl;
}


//saves image to filename given as argument. Warning, this overwrites the file without warning!. at the moment, only one sample per picture
void Film::convert() {
    for (int y=0; y<height; y++){
        for (int x=0; x<width; x++){
            double count = buckets[width * y + x].n;
            image[4 * width * y + 4 * x + 0] = fmin(buckets[width * y + x].c.getRed() / count, 255.0f); //RED
            image[4 * width * y + 4 * x + 1] = fmin(buckets[width * y + x].c.getGreen() / count, 255.0f);
            image[4 * width * y + 4 * x + 2] = fmin(buckets[width * y + x].c.getBlue() / count, 255.0f);
            image[4 * width * y + 4 * x + 3] = 255.0; //ALPHA for controlling transaperncy: MAINTAIN AT 255
        }
    }
}

#endif