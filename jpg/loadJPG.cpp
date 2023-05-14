#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "loadJPG.h"
#include <jpeglib.h> 
#pragma comment(lib, "jpeg.lib")
#define BITS_PER_CHANNEL_8 8
#define BITS_PER_CHANNEL_16 16
#define IMAGE_IO_RGB 3
#define IMAGE_IO_RGB_ALPHA 4
#define IMAGE_IO_HORIZONTAL_DIFFERENCING 2
#define IMAGE_IO_UNCOMPRESSED_RGB 2

LoadJPG::LoadJPG() {
    width = 0;
    height = 0;
    bytesPerPixel = 0;
    pixels = nullptr;
    ownPixels = 0;
}

LoadJPG::LoadJPG(unsigned int width_, unsigned int height_, 
    unsigned int bytesPerPixel_, unsigned char* pixels_, int makeInternalCopy) :
    width(width_), height(height_), bytesPerPixel(bytesPerPixel_) {
    if (makeInternalCopy == 0) {
        pixels = pixels_;
        ownPixels = 0;
    }
    else {
        pixels = (unsigned char*)malloc(sizeof(unsigned char) * width * height * bytesPerPixel);
        memcpy(pixels, pixels_, sizeof(unsigned char) * width * height * bytesPerPixel);
        ownPixels = 1;
    }
}

LoadJPG::~LoadJPG() {
    if (ownPixels)
        free(pixels);
}

LoadJPG::errorType LoadJPG::load(const char* filename) {
    FILE* file = fopen(filename, "rb");
    if (!file)
        return IO_ERROR;

    struct jpeg_decompress_struct jpgPicture;
    struct jpeg_error_mgr jpgErrorMessage;

    jpgPicture.err = jpeg_std_error(&jpgErrorMessage);
    j_decompress_ptr jpgPicturePtr = (j_decompress_ptr)(&jpgPicture);

    jpeg_create_decompress(jpgPicturePtr);  // Init
    jpeg_stdio_src(jpgPicturePtr, file);  // setup image source

    // read information of the image
    jpeg_read_header(jpgPicturePtr, TRUE);
    jpeg_start_decompress(jpgPicturePtr);

    width = jpgPicturePtr->image_width;
    height = jpgPicturePtr->image_height;

    // CAREFULL we must be!! the bytesPerPixel information is not in the jpeg header.
    // It is only available after calling jpeg_start_decompress()
    bytesPerPixel = jpgPicturePtr->output_components;

    free(pixels);
    pixels = (unsigned char*)malloc(sizeof(unsigned char) * width * height * bytesPerPixel);
    //  printf("Width = %d; Height = %d, bytesPerPixel = %d\n", width, height, bytesPerPixel);
    //  fflush(nullptr);

    JSAMPROW rowPtr[1];
    for (int row = jpgPicturePtr->output_height - 1; 
        jpgPicturePtr->output_scanline < jpgPicturePtr->output_height; row--){
        rowPtr[0] = (JSAMPROW)&pixels[row * width * bytesPerPixel];
        JDIMENSION maxNumLines = 1;
        if (jpeg_read_scanlines(jpgPicturePtr, (JSAMPARRAY)rowPtr, maxNumLines) != maxNumLines) {
            printf("Error in loadJPEG: Error reading jpg image from %s.\n", filename);
            free(pixels);
            jpeg_destroy_decompress(jpgPicturePtr);
            fclose(file);
            return IO_ERROR;
        }
    }
    jpeg_finish_decompress(jpgPicturePtr);
    jpeg_destroy_decompress(jpgPicturePtr);
    fclose(file);
    return OK;
}

LoadJPG::errorType LoadJPG::save(const char* filename) {
    int quality = 95;
    FILE* file = fopen(filename, "wb");
    if (!file)
        return IO_ERROR;

    struct jpeg_compress_struct jpgPicture;
    struct jpeg_error_mgr jpgErrorMessage;

    jpgPicture.err = jpeg_std_error(&jpgErrorMessage);
    jpeg_create_compress(&jpgPicture);   // Init
    jpeg_stdio_dest(&jpgPicture, file);  // setup image destination

    unsigned char* pixelsNoAlphaChannel = nullptr;
    if (bytesPerPixel == 4) { // special case, alpha channel byte will be dropped
        printf("Warning in saveJPEG: Alpha channel has been dropped when the image is saved in JPEG format.\n");
        pixelsNoAlphaChannel = (unsigned char*)malloc(sizeof(unsigned char) * width * height * 3);
        for (unsigned int pixelIndex = 0; pixelIndex < width * height; pixelIndex++)
            memcpy(&pixelsNoAlphaChannel[pixelIndex * 3], &pixels[pixelIndex * 4], sizeof(unsigned char) * 3);
    }
    else
        pixelsNoAlphaChannel = pixels;

    jpgPicture.image_width = width;
    jpgPicture.image_height = height;
    jpgPicture.input_components = 3;
    jpgPicture.in_color_space = JCS_RGB; 	// color space of the image

    jpeg_set_defaults(&jpgPicture);
    jpeg_set_quality(&jpgPicture, quality, TRUE);
    jpeg_start_compress(&jpgPicture, TRUE);

    JSAMPROW rowPtr[1];
    unsigned int numBytesPerRow = width * 3;
    for (int row = jpgPicture.image_height - 1; 
        jpgPicture.next_scanline < jpgPicture.image_height; row--) {
        rowPtr[0] = &pixelsNoAlphaChannel[row * numBytesPerRow];
        JDIMENSION maxNumLines = 1;
        if (jpeg_write_scanlines(&jpgPicture, rowPtr, maxNumLines) != maxNumLines) {
            printf("Error in saveJPEG: Error while saving jpg image to %s.\n", filename);
            jpeg_destroy_compress(&jpgPicture);
            if (bytesPerPixel == 4)
                free(pixelsNoAlphaChannel);
            fclose(file);
            return IO_ERROR;
        }
    }
    jpeg_finish_compress(&jpgPicture);
    jpeg_destroy_compress(&jpgPicture);
    fclose(file);
    if (bytesPerPixel == 4)
        free(pixelsNoAlphaChannel);
    return OK;
}

