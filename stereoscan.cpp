//
// Created by Zahorack (Oliver Holl�)
//

#include "stereoscan.h"


static void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist, cv::Rect roi);
static float get_depth_scale(rs2::device dev);
static rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
static bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
static void invert(rs2::video_frame& color_frame, const rs2::depth_frame& depth_frame);



Stereoscan::Stereoscan(rs2::pipeline& pipe) :
    m_pipeline(pipe),
    m_profile(m_pipeline.start()),
    depth_scale(get_depth_scale(m_profile.get_device())),
    align_to(find_stream_to_align(m_profile.get_streams())),
    aligninig(align_to)
{
    auto profile = pipe.get_active_profile();
    if (profile) {
        std::cout << "Device OK";
    }
    else {
        std::cout << "Device disconnected";
    }

}

Stereoscan::~Stereoscan()
{
    m_pipeline.stop();
}



void Stereoscan::update()
{
    FaceDetection faceDetection;

    do {
        m_data = m_pipeline.wait_for_frames();
      
        
        static unsigned long long last_frame_number = 0;
        if (m_data.get_frame_number() == last_frame_number)
            continue;
        last_frame_number = m_data.get_frame_number();

        rs2::video_frame color_frame = m_data.get_color_frame();

        auto color_mat = frame_to_mat(color_frame);
        

        cv::flip(color_mat, color_mat, -1);
        faceDetection.update(color_mat);

    } while (!faceDetection.available());

    process(faceDetection);
}

void Stereoscan::initialize() {

}

void Stereoscan::process(FaceDetection faceDetection) {
    rs2::video_frame color_frame = m_data.get_color_frame();
    rs2::depth_frame depth_frame = m_data.get_depth_frame();


    invert(color_frame, depth_frame);

    auto color_mat = frame_to_mat(color_frame);
    auto depth_mat = frame_to_mat(depth_frame);

    if (profile_changed(m_pipeline.get_active_profile().get_streams(), m_profile.get_streams()))
    {
        //If the profile was changed, update the align object, and also get the new device's depth scale
        m_profile = m_pipeline.get_active_profile();
        align_to = find_stream_to_align(m_profile.get_streams());
        aligninig = rs2::align(align_to);
        depth_scale = get_depth_scale(m_profile.get_device());
    }

    auto faces = faceDetection.crops(1.4, 1.3);
    auto centers = faceDetection.centers();


    for (auto i = 0; i < faces.size(); i++) {

        std::cout << "face: " << i <<std::endl;

        auto processed = aligninig.process(m_data);

        rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame) {
            continue;
        }

        auto center_distance = depth_frame.get_distance(centers[i].x, centers[i].y);
        float depth_clipping_distance = center_distance + static_cast<float>(0.1);
        remove_background(other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance, faces[i]);

        //SAVING
        rs2::pointcloud pc;
        pc.map_to(other_frame);
        rs2::points points = pc.calculate(aligned_depth_frame);

        std::string filename = "faces";
        std::stringstream ssf;
        ssf << "faces/3D/" << filenumber << ".ply";
        points.export_to_ply(ssf.str(), color_frame);

        auto other_mat = frame_to_mat(other_frame);
        std::stringstream ssfn;
        ssfn << "faces/RGB/" << filenumber << ".jpg";
        filename = ssfn.str();
        cv::imwrite(filename, other_mat(faces[i]));
        filenumber++;


        cv::Point pt1(faces[i].x, faces[i].y); // Display detected faces on main window
        cv::Point pt2((faces[i].x + faces[i].width), (faces[i].y + faces[i].height));
        rectangle(color_mat, pt1, pt2, cv::Scalar(0, 255, 0), 2, 8, 0);
    }


    cv::imshow("image", color_mat);
}


void Stereoscan::save() {

}

static float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}


static rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}


static void remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist, cv::Rect roi)
{
    const uint16_t* p_depth_frame_to_read = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));

    uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    int width = other_frame.get_width();
    int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();
    int deep_bpp = depth_frame.get_bytes_per_pixel();

    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;

#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            auto pixels_distance = depth_scale * p_depth_frame_to_read[depth_pixel_index];
            // Calculate the offset in other frame's buffer to current pixel
            auto offset = depth_pixel_index * other_bpp;

            if (x > roi.x&& x < (roi.x + roi.width) && y > roi.y&& y < (roi.y + roi.height)) {
                if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
                {
                    p_depth_frame[depth_pixel_index] = 0;
                    // Set pixel to "background" color (0x999999)
                    std::memset(&p_other_frame[offset], 255, other_bpp);
                }
            }
            else {
                p_depth_frame[depth_pixel_index] = 0;
                // Set pixel to "background" color (0x999999)
                //std::memset(&p_other_frame[offset], 255, other_bpp);
            }
            // Check if the depth value is invalid (<=0) or greater than the threashold
        }
    }
}

template<typename T>
static void revereseArray(T* start, T* stop, int elemnetSize) {

    T* head = start;
    T* tail = stop;


    while (head < tail) {
        T temp[elemnetSize];

        std::memcpy(temp, head, elemnetSize);
        std::memcpy(head, tail, elemnetSize);
        std::memcpy(tail, temp, elemnetSize);

        head += elemnetSize;
        tail -= elemnetSize;
    }

}

static void invert(rs2::video_frame& color_frame, const rs2::depth_frame& depth_frame)
{
    const uint16_t* p_depth_frame_to_read = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));

    uint8_t* p_color_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(color_frame.get_data()));

    int width = color_frame.get_width();
    int height = color_frame.get_height();
    int color_bpp = color_frame.get_bytes_per_pixel();
    int deep_bpp = depth_frame.get_bytes_per_pixel();


    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;

    std::cout << "deep_bpp: " << deep_bpp << std::endl;

    const auto color_frame_size = width * height * color_bpp;
    const auto depth_frame_size = depth_frame.get_width() * depth_frame.get_height();

    std::cout << "depth_frame: " << depth_frame.get_width() << std::endl;
    std::cout << "depth_frame: " << depth_frame.get_height() << std::endl;

    uint8_t* color_buff = new uint8_t[color_frame_size];
    uint16_t* depth_buff= new uint16_t[depth_frame_size];

    //std::memcpy(color_buff, p_color_frame+50, color_frame_size);
    //std::memcpy(depth_buff, p_depth_frame, depth_frame_size);

    for (int i = 0; i < depth_frame_size; i++) {
        depth_buff[i] = p_depth_frame[i];
    }

    for (int i = 0; i < color_frame_size; i++) {
        color_buff[i] = p_color_frame[i+51];
    }


   //std::reverse(p_color_frame+50, p_color_frame + color_frame_size);
   // std::reverse(p_depth_frame, p_depth_frame + depth_frame_size);

   int yc = 0;
//#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (yc = 0; yc < height; yc++)
    {

        //std::cout << yc << std::endl;
        auto depth_pixel_index = yc * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Calculate the offset in other frame's buffer to current pixel
            auto offset = depth_pixel_index * color_bpp;

           // std::cout << "i1: " << depth_pixel_index << "  i2:  " << depth_frame_size - depth_pixel_index << std::endl;
            p_depth_frame[depth_pixel_index] = depth_buff[depth_frame_size- depth_pixel_index ];
            //p_depth_frame[depth_pixel_index] = depth_buff[depth_pixel_index];
            //p_color_frame[offset] = color_buff[color_frame_size - offset];

           // std::memcpy(&p_8_depth_frame[depth_pixel_index], &depth_buff[depth_frame_size - depth_pixel_index], 2);

            std::memcpy(&p_color_frame[offset], &color_buff[color_frame_size - offset], color_bpp);
        }
    }

    delete[] color_buff;
    delete[] depth_buff;
}




static bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}