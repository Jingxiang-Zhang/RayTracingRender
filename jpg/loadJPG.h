#ifndef _LOADJPG_H_
#define _LOADJPG_H_

class LoadJPG
{
public:
	LoadJPG();
	// makeInternalCopy:
	// 0: do not copy pixels internally (shallow copy)
	// 1: copy pixels internally (deep copy)
	LoadJPG(unsigned int width, unsigned int height, unsigned int bytesPerPixel, 
		unsigned char* pixels, int makeInternalCopy = 1);
	~LoadJPG();

	inline unsigned int getWidth() { return width; }
	inline unsigned int getHeight() { return height; }
	inline unsigned int getBytesPerPixel() { return bytesPerPixel; }
	inline unsigned char* getPixels() { return pixels; }
	inline unsigned char getPixel(int x, int y, int channel) { return pixels[(y * width + x) * bytesPerPixel + channel]; }

	// error codes
	typedef enum { OK, INVALID_FILE_FORMAT, IO_ERROR, MEMORY_ERROR, OTHER_ERROR } errorType;
	errorType load(const char* filename);
	errorType save(const char* filename);
protected:
	unsigned int width, height;
	unsigned int bytesPerPixel;
	unsigned char* pixels;
	int ownPixels;
};

#endif

