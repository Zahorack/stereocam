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

    std::cout << "Logger begin on " << m_date << std::endl;
    std::cout << "Logger begin at " << m_time << std::endl;

}

Logger::~Logger() {}


void Logger::updateRGB(cv::Mat& frame) {

    const std::string dir = "RGB\\";
    CreateDirectoryA((PATH + dir + today()).c_str(), NULL);

    std::stringstream ssfn;
    ssfn << PATH << "RGB\\" << today() << "\\" << now() <<".jpg";
    std::cout << ssfn.str() << std::endl;
    auto filename = ssfn.str();

    cv::imwrite(filename,frame);
}

void Logger::update3D(rs2::video_frame& color, rs2::depth_frame& depth) {

    const std::string dir = "3D\\";
    CreateDirectoryA((PATH + dir + today()).c_str(), NULL);

    rs2::pointcloud pc;
    pc.map_to(color);
    rs2::points points = pc.calculate(depth);

    std::stringstream ssf;
    ssf << PATH << "3D\\"<< today() <<"\\"<< now() << ".ply";
    points.export_to_ply(ssf.str(), color);
}