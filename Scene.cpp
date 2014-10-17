// Scene.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "lodepng.h"
#include <iostream>
#include <Eigen/Core>

#include <Eigen/Dense>
using namespace Eigen;
using namespace std;


//CODE FOR WRITING TO AN IMAGE FILE (LODEPNG LIBRARY), 
//CITATION: ALL CODE BELOW, specifically writeimage and commit methods, ADAPTED/BORROWED FROM THE EXAMPLE FILE ON THE LODEPNG WEBSITE. 
//I DO NOT ASSUME 100% CREDIT FOR THAT CODE. ALL DUE CREDIT IS GIVEN TO THE MAKERS OF THE LIBRARY. 

//"g++ lodepng.cpp example_encode.cpp -ansi -pedantic -Wall -Wextra -O3"

//From lodepng example: "Encode from raw pixels to disk with a single function call"
//"The image argument has width * height RGBA pixels or width * height * 4 bytes"


//FILM CLASS
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

 //END FILM CLASS


 //GLOBAL VARIABLES. EYE AND FOCAL PLANE
 Vector3f eye(50.0f, 50.0f, -300.0f); 
 float focal_plane=0.0f; 


 //RAY CLASS
 class Ray{
	public:
	 Vector3f source;
	 Vector3f direction;
	 float t_min, t_max;
	 Ray(Vector3f src, Vector3f dir, float min, float max);
 };

 Ray::Ray(Vector3f src, Vector3f dir, float min, float max){
	 source = src;
	 direction = dir;
	 t_min = min;
	 t_max = max; 
 }
 
 Ray generateRay(float i, float j){
	 Vector3f pixel_loc = Vector3f(i + .5f, j + .5f, focal_plane);
	 Vector3f direction = pixel_loc - eye;
	 direction.normalize();
	 ///FIX ME
	 Ray q(eye, direction, focal_plane, 10000000.0f);//MAY NEED TO BE INCREASED
	 return q; 
	 //END FIX ME
 }
 
 //END RAY CLASS

 class Color{
 public:
	 float red;
	 float blue; 
	 float green;
	 Color(float r, float g, float b);
 };

 Color::Color(float r, float g, float b){
	 red = r;
	 green = g;
	 blue = b; 
 }


//SCENE CLASS
 class Scene{
	public:
		Film negative;
		float height;
		float width;
		float depth;

 };
//END SCENE CLASS

 //Sphere class
 class Sphere{
 public:
	 float radius;
	 Vector3f center;
	 Sphere(Vector3f, float);
	 int hit(Ray r);
 };

 Sphere::Sphere(Vector3f c, float r){
	 center = c;
	 radius = r; 
 }

 int Sphere::hit(Ray r){
	 Vector3f d = r.direction;
	 Vector3f e = r.source;
	 Vector3f c = center; 
	 Vector3f ec = e - c; 

	 float term1 = (d.dot(ec))*(d.dot(ec));
	 float term2 = (d.dot(d))*((ec).dot(ec) - radius*radius);
	 float discr = term1 - term2; 
	 if (discr >= 0.0f){
		 return 1;
	 }
	 return 0; 
 }

 Color trace(Ray ray, Sphere sphere){
	 float red = 200;
	 float blue = 100;
	 float green = 155;
	 if (sphere.hit(ray)==1){
		 return Color(red, blue, green);
	 }
	 return Color(0.0f, 0.0f, 0.0f);
 }



 int main(int argc, char *argv[]){
	 int height = 100;
	 int width = 100; 
	 Film negative(width, height);
	 Sphere test(Vector3f(50.0f, 50.0f, 150.0f), 50.0f);
	 for (int i = 0; i < width; i++){
		 for (int j = 0; j < height; j++){
			 Ray temp = generateRay(i, j);
			 Color result = trace(temp, test);
			 negative.commit(i, j, result.red, result.green, result.blue);
			 //negative.commit(i, j, 255*i/width, 255*j/height, 200) activate this for pretty colors
		 }
	 }
	 negative.writeImage();
	
	 /*
	 Vector3f v(1.0f, 2.0f, 3.0f);
	 Vector3f w(0.0f, 1.0f, 2.0f);
	 int dotty = v.dot(w);
	 Vector3f sub = v - w; 
	 */
	// Sphere test(Vector3f(5.0f, 5.0f, 5.0f), 5.0f);
	// Ray trial(Vector3f(5.0f+3.0f,5.0f+4.1f, 0.0f), Vector3f(0.0f, 0.0f, 1.0f), 1.0f, 300.0f);
	 //int hh = test.hit(trial);
	// cout << "Dot product: " << hh << endl;
	 

	 return 0; 
 }







 /*

 class Parent {

 public:
 void doShit()
 {
 cout << "message " << "parent" << endl;
 }

 };


 class Child: public Parent {

 public:
 void doShit()
 {
 cout << "message " << "child" << endl;
 }
 */
 





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
