// Scene.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "lodepng.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <Eigen/Core>

#include <Eigen/Dense>
using namespace Eigen;
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
		image[4 * width * y + 4 * x + 3] = 255; //ALPHA for controlling transaperncy: MAINTAIN AT 255
 }

 //END FILM CLASS


 //GLOBAL VARIABLES. EYE AND FOCAL PLANE
 Vector3f eye(0.0f, 0.0f, 0.0f); 
 Vector3f UL(-1.0f, 1.0f, -1.0f);
 Vector3f UR(1.0f, 1.0f, -1.0f);
 Vector3f LL(1.0f, -1.0f, -1.0f);
 Vector3f LR(-1.0f, -1.0f, -1.0f);
 Vector3f xvec; // horizontal basis vector of focal plane
 Vector3f yvec; // vertical basis vector of focal plane

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
	 float focalplane = (UL +(xvec * i / 100)+(yvec * j / 100))[2];
	 Vector3f pixel_loc = Vector3f(UL[0]+xvec[0]*i/100,UL[0]+ yvec[1]*j/100,focalplane );
	 Vector3f direction = pixel_loc - eye;
	 direction.normalize();
	 ///FIX ME
	 Ray q(eye, direction, focalplane, 10000000.0f);//MAY NEED TO BE INCREASED
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
	 Color operator +(Color);
 };

 Color::Color(float r, float g, float b){
	 red = r;
	 green = g;
	 blue = b; 
 }

 Color Color::operator +(Color param) {
	 return Color(param.red + red, param.blue + blue, param.green + green);
 }

 //Class for directional lights
 class Dlight{
 public:
	 Vector3f direction;
	 float red;
	 float blue;
	 float green;
 };

 //Class for point lights
 class Plight{
 public:
	 Vector3f source;
	 float red;
	 float blue;
	 float green;
	 Plight(Vector3f src, float r, float g, float b);
 };

 Plight::Plight(Vector3f src, float r, float g, float b){
	 source = src;
	 red = r;
	 blue = b;
	 green = g; 
 }


 vector <Plight> plights;
 vector <Dlight> dlights;
 


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

 class Surface{
 public:
	 Vector3f specularcoef;
	 Vector3f diffusecoef; 
	 virtual int hit(Ray * r) = 0 ;
	 virtual Color diffuse(Ray * r, float t)=0; 
	 virtual Color specular(Ray * r, float t)=0; 
	 virtual float intersect(Ray *r) = 0; 
 };

 class Sphere: public Surface{
 public:
	 float radius;
	 Vector3f center;
	 Sphere(Vector3f, float, Vector3f, Vector3f);
	 int hit(Ray * r);
	 Color diffuse(Ray * r, float t);
	 Color specular(Ray * r, float t);
	 float intersect(Ray *r);
 };


 Sphere::Sphere(Vector3f c, float rad, Vector3f diff, Vector3f spec){
	 center = c;
	 radius = rad; 
	 diffusecoef = diff;
	 specularcoef = spec; 
 }

 int Sphere::hit(Ray  * r){
	 Vector3f d = r->direction;
	 Vector3f e = r->source;
	 Vector3f c = center; 
	 Vector3f ec = e - c; 

	 float term1 = (d.dot(ec))*(d.dot(ec));
	 float term2 = (d.dot(d))*((ec).dot(ec) - radius*radius);
	 float discr = term1 - term2; 

	 if (discr >= 0){
		 return 1;
	 }
	 return 0; 
 }

 float Sphere::intersect(Ray * r){
	 Vector3f d = r->direction;
	 Vector3f e = r->source;
	 Vector3f c = center;
	 Vector3f ec = e - c;

	 float term1 = (d.dot(ec))*(d.dot(ec));
	 float term2 = (d.dot(d))*((ec).dot(ec) - radius*radius);
	 float discr = term1 - term2;

	 float t1 = (-(d.dot(ec)) - sqrt(discr)) / (d.dot(d));
	// float t2 = (-(d.dot(ec)) + sqrt(discr)) / (d.dot(d));

	 return t1;

 }

 Color Sphere::diffuse(Ray * r, float t){
	 Color rgb(0.0f, 0.0f, 0.0f); 
	 for (int a = 0.0f; a < plights.size(); a++){
		 
		 Vector3f surfacepoint = r->source + t*(r->direction);
		 Vector3f l = plights[a].source - surfacepoint;
		 l.normalize();
		 
		 Vector3f n = surfacepoint - center;
		 n.normalize();

		 float result = n.dot(l);
		 rgb.red += max(result, 0.0f)*diffusecoef[0]*plights[a].red;
		 rgb.green += max(result, 0.0f)*diffusecoef[1]*plights[a].blue;
		 rgb.blue += max(result, 0.0f)*diffusecoef[2]*plights[a].green;
	 }
	 //cout << "Dot product: " << rgb.red<< "blue" << rgb.blue<< endl;
	 return rgb;
 }

 Color Sphere::specular(Ray * r, float t){

	 Color rgb(0.0f, 0.0f, 0.0f);
	 for (int a = 0.0f; a < plights.size(); a++){

		 Vector3f surfacepoint = r->source + t*(r->direction);
		 Vector3f l = plights[a].source - surfacepoint;
		 l.normalize();

		 Vector3f n = surfacepoint - center;
		 n.normalize();

		 Vector3f r = Vector3f(0.f, 0.0f, 0.0f)+l - (2 * (l.dot(n)))*(n);
		// r.normalize(); 

		 Vector3f v = surfacepoint-eye;
		 v.normalize();

		 float result = r.dot(v);

		 float p = 2;

		 if (result > 0.0f){
			 float result2 = powf(result, p);
			 rgb.red += result2*specularcoef[0]*plights[a].red;
			 rgb.blue += result2*specularcoef[1]*plights[a].blue;
			 rgb.green += result2*specularcoef[2]*plights[a].green;
		 }

		 /*
		 rgb.red += pow(max(result, 0.0f), p)*specularcoef[0] * plights[a].red;
		 rgb.green += pow(max(result, 0.0f), p)*specularcoef[1] * plights[a].blue;
		 rgb.blue += pow(max(result, 0.0f), p)*specularcoef[2] * plights[a].green;
		 */
	 }
	 //cout << "Dot product: " << rgb.red<< "blue" << rgb.blue<< endl;
	 //return Color(0.0f, 0.0f, 0.0f);
	 return rgb;
 }

 vector<Sphere> balls;


 Color trace1(Ray * ray, Sphere * sphere){
	 float red = 200;
	 float blue = 100;
	 float green = 155;
	 if (sphere->hit(ray)==1){
		 float intersect = sphere->intersect(ray);
		 return sphere->diffuse(ray, intersect) + sphere->specular(ray, intersect);
		 //return Color(red, blue, green);
	 }
	 return Color(0.0f, 0.0f, 0.0f);
 }

 Color trace(Ray * ray, vector<Sphere>  balls){

	 float closest_t = 999999999999999999; //FIX ME MAX FLOAT
	 Sphere * closest_sphere = 0; 
	 for (int x = 0; x < balls.size(); x++){
		 Sphere  sphere = ((balls)[x]);
		 if (sphere.hit(ray) == 1){
			 float intersect = sphere.intersect(ray);
			 if (intersect < closest_t){
				 closest_t = intersect;
				 closest_sphere = &((balls)[x]);
			 }
		 return closest_sphere->diffuse(ray, closest_t) + closest_sphere->specular(ray, closest_t);
		 //return Color(red, blue, green);
		}
	}
	 return Color(0.0f, 0.0f, 0.0f);
 }



 class Parent {

 public:
	 virtual void doShit() = 0;

 };


 class Child : public Parent {

 public:
	 void doShit()
	 {
		 cout << "message " << "child" << endl;
	 }
 };

 class Child1 : public Parent {

 public:
	 void doShit()
	 {
		 cout << "message " << "child1" << endl;
	 }
 };




 int main(int argc, char *argv[]){
	 int height = 200;
	 int width = 200; 

	 xvec = UR - UL;
	 yvec = UL - LL;


	 Film negative(width, height);
	 Vector3f rgbc(1.0f, 1.0f, 1.0f);
	 Vector3f rgbd(0.0f, 0.4f, 0.3f);
	 Sphere  test=Sphere(Vector3f(0.0f, .0f, -2.0f), 1.0f, rgbc, rgbd);
	 balls.push_back(test);
	 Sphere  test2 = Sphere(Vector3f(1.0f, 0.0f, -2.5f), 1.0f, Vector3f(1.0f, 0.0f, 1.0f), Vector3f(0.3f, 0.5f, 0.2f));
	 balls.push_back(test2);

	 Plight source(Vector3f(2.0f, 2.0f, 2.0f), 200.0f, 0.0f, 155);
	 plights.push_back(source);

	 for (int i = 0; i < width; i++){
		 for (int j = 0; j < height; j++){
			 Ray * temp = &generateRay(i, j);
			 Color * result = &trace(temp, balls);
			 negative.commit(i, j, result->red, result->green, result->blue);
			 //FIX ME: currently only supporting one sample per pixel
			 //negative.commit(i, j, 255*i/width, 255*j/height, 200) activate this for pretty colors
		 }
	 }

	 negative.writeImage();
	
	 //testing random shit
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


	 //polymorphism dynamic late binding check: should output child1 and child2
	 Child child;
	 Child1 child1;
	 Parent * a = &child;
	 Parent * b = &child1;
	 a->doShit();
	 b->doShit();


	 float num = UL[0];
	 cout << "Dot product: " << num << endl;
	 system("PAUSE");

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
