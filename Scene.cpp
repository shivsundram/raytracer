// Scene.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "lodepng.h"
#include <iostream>


//CODE FOR WRITING TO AN IMAGE FILE (LODEPNG LIBRARY), 
//CITATION: ALL CODE BELOW, specifically writeimage and commit methods, ADAPTED/BORROWED FROM THE EXAMPLE FILE ON THE LODEPNG WEBSITE. 
//I DO NOT ASSUME 100% CREDIT FOR THAT CODE. ALL DUE CREDIT IS GIVEN TO THE MAKERS OF THE LIBRARY. 

//"g++ lodepng.cpp example_encode.cpp -ansi -pedantic -Wall -Wextra -O3"

//From lodepng example: "Encode from raw pixels to disk with a single function call"
//"The image argument has width * height RGBA pixels or width * height * 4 bytes"


//
class Film{
	public:
		Film(int width, int height);
		int width; 
		int height; 
		std::vector<unsigned char> image;
		void writeImage();
		void commit(int x, int y, float r, float g, float b);
};

Film::Film(int width1, int height1){
	width = width1;
	height = height1;
	image.resize(width * height * 4);
}

 void Film::writeImage()
{
	 //NOTE: this sample will overwrite the file or rayprint.png without warning!
	 const char* filename = "rayprint.png";

	//Encode the image
	unsigned error = lodepng::encode(filename, image, width, height);

	//if there's an error, display it
	if (error) std::cout << "encoder error " << error << ": " << lodepng_error_text(error) << std::endl;
}


//saves image to filename given as argument. Warning, this overwrites the file without warning!. at the moment, only one sample per picture
 void Film::commit(int x, int y, float r, float g, float b)
{
		image[4 * width * y + 4 * x + 0] = r; //RED 
		image[4 * width * y + 4 * x + 1] = g; //GREEN
		image[4 * width * y + 4 * x + 2] = b; //BLUE
		image[4 * width * y + 4 * x + 3] = 255; //AOLPHA: MAINTAIN AT 255
 }

 class Scene{
	public:
		Film negative;
		float height;
		float width;
		float depth;

 };




 int main(int argc, char *argv[]){
	 int height = 512;
	 int width = 512; 
	 Film negative(width, height);
	 
	 for (unsigned i = 0; i < width; i++){
		 for (unsigned j = 0; j < height; j++){
			 //generate sample
			 negative.commit(i, j, 200, 200, 200);
		 }
	 }
	 negative.writeImage();
	 
	 return 0; 
 }





//Andrey, the following is SHIT WE DO NOT NEED. ALSO FROM LODEPNG WEBSITE. ALTERNATE WAYS OF ENCODING IMAGE

//Example 2
//Encode from raw pixels to an in-memory PNG file first, then write it to disk
//The image argument has width * height RGBA pixels or width * height * 4 bytes
void encodeTwoSteps(const char* filename, std::vector<unsigned char>& image, unsigned width, unsigned height)
{
	std::vector<unsigned char> png;

	unsigned error = lodepng::encode(png, image, width, height);
	if (!error) lodepng::save_file(png, filename);

	//if there's an error, display it
	if (error) std::cout << "encoder error " << error << ": " << lodepng_error_text(error) << std::endl;
}

//Example 3
//Save a PNG file to disk using a State, normally needed for more advanced usage.
//The image argument has width * height RGBA pixels or width * height * 4 bytes
void encodeWithState(const char* filename, std::vector<unsigned char>& image, unsigned width, unsigned height)
{
	std::vector<unsigned char> png;
	lodepng::State state; //optionally customize this one

	unsigned error = lodepng::encode(png, image, width, height, state);
	if (!error) lodepng::save_file(png, filename);

	//if there's an error, display it
	if (error) std::cout << "encoder error " << error << ": " << lodepng_error_text(error) << std::endl;
}
