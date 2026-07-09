#include "camera_frame.h"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace
{
void require (bool condition, const char* message)
{
  if( !condition )
  {
    throw std::runtime_error (message);
  }
}

Camera_Frame make_frame (std::uint8_t value)
{
  Camera_Frame frame;
  Camera_Image_Frame image;
  image.width = 1;
  image.height = 1;
  image.data = { value };
  frame.images.push_back (std::move (image));
  return frame;
}
} // namespace

int main ( )
{
  try
  {
    Camera_Frame_Buffer buffer;
    require (!buffer.Latest ( ), "New frame buffer should be empty");

    buffer.Publish (make_frame (1));
    const auto first = buffer.Latest ( );
    require (first != nullptr, "First frame was not published");
    require (first->generation == 1, "First generation mismatch");
    require (first->images[0].data[0] == 1, "First frame data mismatch");

    std::thread producer (
      [&buffer]
      {
        for( std::uint16_t value = 2; value <= 100; ++value )
        {
          buffer.Publish (make_frame (static_cast<std::uint8_t> (value)));
        }
      });
    producer.join ( );

    const auto latest = buffer.Latest ( );
    require (latest != nullptr, "Latest frame is missing");
    require (latest->generation == 100, "Latest generation mismatch");
    require (latest->images[0].data[0] == 100, "Latest frame data mismatch");
    require (first->images[0].data[0] == 1, "Published frame was not immutable");

    buffer.Clear ( );
    require (!buffer.Latest ( ), "Frame buffer clear failed");
  }
  catch( const std::exception& error )
  {
    std::cerr << "FAILED: " << error.what ( ) << '\n';
    return 1;
  }

  std::cout << "Camera frame buffer tests passed.\n";
  return 0;
}
