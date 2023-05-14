#include "model/load_data.h"
#include "openGL/plot.h"
#include <stdio.h>
#include <random>
#include <ctime>
#include "algorithm/rayTracing.h"
#include "command.h"

int main(int argc, char** argv)
{
    cmd_parameter(argc, argv);
    // mode 1: Phong shading
    // you can use the following path: "spheres.scene", "test1.scene", "test2.scene", 
    // "toy.scene", "snow.scene", "SIGGRAPH.scene", "table.scene"
    // you also need to set the render mode:
    // startParameter.mode = RenderMode::BRDF;

    // mode 2: BRDF shading with area light
    // you can use the following path: "siggraph_BRDF.scene", "snow-man_BRDF.scene",
    // "table_BRDF.scene", "test2_BRDF.scene"
    // you also need to set the render mode:
    // startParameter.mode = RenderMode::BRDF;

    // set up common parameter, the parameter below will be used by both of the 2 modes
    {
        mode = MODE_JPEG; // save the image
        img_savepath = startParameter.img_savepath; // set image save path
        AntiAliasing = SuperSampling::X4; // set antialiasing
        global_picker_setting.maximum_reflection = 1; // set max light reflection

        verbose = false; // disable output when load the scene
        // the distance attenuation will be:
        // attenuation_coef_t / (attenuation_coef_a * d ^ 2 + attenuation_coef_b * d + 
        // attenuation_coef_c)
        attenuation_coef_t = 100.0;
        attenuation_coef_a = 1.0;
        attenuation_coef_b = 0;
        attenuation_coef_c = 100.0;
    }
    // if the mode is Phong shading, use the following setting
    if (startParameter.mode == RenderMode::PhoneShading) {
        attenuation_reflection = 0.2;
        attenuation_diffusion = 0.2;
        // can also set Phone illumination coef, if uncomment it, then use default setting.
        // La[3];
        // Ld[3];
        // Ls[3];
        // ka[3];
        // for each ray, spread to n_random_spread random directions
        global_picker_setting.n_random_spread = 100;
        loadScenePhong(startParameter.file_path); // load Phong mode scene
        renderMode = RenderModeRT::PhoneShading; // set render mode
    }
    if (startParameter.mode == RenderMode::BRDF) {
        loadSceneBRDF(startParameter.file_path); // load BRDF mode scene
        renderMode = RenderModeRT::BRDF; // set render mode
        global_picker_setting.n_light_sampling = 100; // random sampling 100 light point
        // set empirical radiation decrease
        attenuation_reflection = 0.5;
        BRDF_light_decrease = 1;
    }
    statistic(); // after loading the data, find the statistic of the scene
    // currently, the scene contain triangle and sphere, use the following function
    // to convert it into object type.
    convert_to_obj();
    init_gl(); // initial OpenGL
    plot(); // start OpenGL main loop
}

