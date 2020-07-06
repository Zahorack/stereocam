//
// Created by Zahorack (Oliver Hollý)
//


#include "Logger.h"
#include <sstream>
#include <chrono>
#include <ctime>  
#include <iomanip>
#include "cv-helpers.h"

#pragma warning(disable : 4996)

static void CreateDir(std::string name) {
    CreateDirectoryA((PATH.c_str() + name).c_str(), NULL);
}


static std::string now()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%H%M%S");
    return ss.str();
}

static std::string today()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d");
    return ss.str();
}

Logger::Logger() 
{
    auto start = std::chrono::system_clock::now();
    std::time_t start_time = std::chrono::system_clock::to_time_t(start);


    m_date = today();
    m_time = now();

    std::cout << "Thermal gate application started on " << m_date;
    std::cout << "  at " << m_time << std::endl;


    const std::string root = PATH;
    CreateDirectoryA((root).c_str(), NULL);

    const std::string rgb = "RGB";
    CreateDirectoryA((PATH + rgb).c_str(), NULL);

    const std::string rgb_faces = "RGB_FACES";
    CreateDirectoryA((PATH + rgb_faces).c_str(), NULL);

    const std::string model = "3D";
    CreateDirectoryA((PATH + model).c_str(), NULL);

    const std::string model_faces = "3D_FACES";
    CreateDirectoryA((PATH + model_faces).c_str(), NULL);

    const std::string thermal = "THERMAL";
    CreateDirectoryA((PATH + thermal).c_str(), NULL);

}

Logger::~Logger() {}


void Logger::updateRGB(cv::Mat& frame, int num) {

    const std::string dir = "RGB\\";
    CreateDirectoryA((PATH + dir + today()).c_str(), NULL);

    std::stringstream ssfn;
    ssfn << PATH << "RGB\\" << today() << "\\" << now() << "_" << num <<".jpg";
    std::cout << ssfn.str() << std::endl;
    auto filename = ssfn.str();

    cv::imwrite(filename,frame);
}

void Logger::updateRGB_FACES(cv::Mat& frame, int num) {

    const std::string dir = "RGB_FACES\\";
    CreateDirectoryA((PATH + dir + today()).c_str(), NULL);

    std::stringstream ssfn;
    ssfn << PATH << "RGB_FACES\\" << today() << "\\" << now() << "_" << num << ".jpg";
    std::cout << ssfn.str() << std::endl;
    auto filename = ssfn.str();

    cv::imwrite(filename, frame);
}

void Logger::update3D(rs2::video_frame& color, rs2::depth_frame& depth, int num) {

    const std::string dir = "3D\\";
    CreateDirectoryA((PATH + dir + today()).c_str(), NULL);

    rs2::pointcloud pc;
    pc.map_to(color);
    rs2::points points = pc.calculate(depth);

    std::stringstream ssf;
    ssf << PATH << "3D\\"<< today() <<"\\"<< now() << "_" << num << ".ply";
    points.export_to_ply(ssf.str(), color);
}

void Logger::update3D_FACES(rs2::video_frame& color, rs2::depth_frame& depth, int num) {

    const std::string dir = "3D_FACES\\";
    CreateDirectoryA((PATH + dir + today()).c_str(), NULL);

    rs2::pointcloud pc;
    pc.map_to(color);
    rs2::points points = pc.calculate(depth);

    std::stringstream ssf;
    ssf << PATH << "3D_FACES\\" << today() << "\\" << now() << "_" << num << ".ply";
    points.export_to_ply(ssf.str(), color);
}
