// Scene.cpp : Defines the entry point for the console application.
//

#ifndef SCENE_H
#define SCENE_H


#include <iostream>

//#include "Film.h"



//class Scene {
//public:
//    Film negative;
//    float height;
//    float width;
//    float depth;
//};



////Andrey, the following is SHIT WE DO NOT NEED. ALSO FROM LODEPNG WEBSITE. ALTERNATE WAYS OF ENCODING IMAGE
//
////Example 2
////Encode from raw pixels to an in-memory PNG file first, then write it to disk
////The image argument has width * height RGBA pixels or width * height * 4 bytes
//void encodeTwoSteps(const char* filename, std::vector<unsigned char>& image, unsigned width, unsigned height)
//{
//	std::vector<unsigned char> png;
//
//	unsigned error = lodepng::encode(png, image, width, height);
//	if (!error) lodepng::save_file(png, filename);
//
//	//if there's an error, display it
//	if (error) std::cout << "encoder error " << error << ": " << lodepng_error_text(error) << std::endl;
//}
//
////Example 3
////Save a PNG file to disk using a State, normally needed for more advanced usage.
////The image argument has width * height RGBA pixels or width * height * 4 bytes
//void encodeWithState(const char* filename, std::vector<unsigned char>& image, unsigned width, unsigned height)
//{
//	std::vector<unsigned char> png;
//	lodepng::State state; //optionally customize this one
//
//	unsigned error = lodepng::encode(png, image, width, height, state);
//	if (!error) lodepng::save_file(png, filename);
//
//	//if there's an error, display it
//	if (error) std::cout << "encoder error " << error << ": " << lodepng_error_text(error) << std::endl;
//}

#endif
