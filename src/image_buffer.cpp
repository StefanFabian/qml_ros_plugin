// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/image_buffer.h"

namespace qml_ros_plugin
{

namespace
{

void iterateImage( const sensor_msgs::Image &img, int bytes_per_pixel,
                   const std::function<void( const uint8_t * )> &func )
{
  const uint8_t *stream = img.data.data();
  int offset = 0;
  for ( unsigned int row = 0; row < img.height; ++row )
  {
    int base = offset;
    for ( unsigned int col = 0; col < img.width; ++col )
    {
      func( stream + base );
      base += bytes_per_pixel;
    }
    offset += img.step;
  }
}

template<typename T>
using Extractor = std::function<T( const uint8_t *, T )>;

template<typename T>
void setFromStream( uint8_t *&output, T val )
{
  *output = val;
  ++output;
}

template<typename T>
bool convertToFormat( const sensor_msgs::Image &img, QVideoFrame::PixelFormat format, uint8_t **data,
                      int &num_bytes, int &bytes_per_line, int bytes_per_pixel,
                      const Extractor<T> &R, const Extractor<T> &G, const Extractor<T> &B, const Extractor<T> &A )
{
  // Reserve array
  switch ( format )
  {
    case QVideoFrame::Format_RGB24:
    case QVideoFrame::Format_BGR24:
      num_bytes = img.width * img.height * 3;
      bytes_per_line = img.width * 3;
      break;
    case QVideoFrame::Format_RGB32:
    case QVideoFrame::Format_ARGB32:
    case QVideoFrame::Format_ARGB32_Premultiplied:
    case QVideoFrame::Format_BGR32:
    case QVideoFrame::Format_BGRA32:
    case QVideoFrame::Format_BGRA32_Premultiplied:
      num_bytes = img.width * img.height * 4;
      bytes_per_line = img.width * 4;
      break;
    case QVideoFrame::Format_Y8:
      num_bytes = img.width * img.height;
      bytes_per_line = img.width;
      break;
    case QVideoFrame::Format_Y16:
      num_bytes = img.width * img.height * 2;
      bytes_per_line = img.width * 2;
      break;
    default:
      qWarning( "Tried to convert to unknown format. This should not be happen! Please open an issue on GitHub." );
      return false;
  }

  *data = new uint8_t[num_bytes];
  switch ( format )
  {
    case QVideoFrame::Format_RGB24:
    {
      auto output = *data;
      iterateImage( img, bytes_per_pixel, [ & ]( const uint8_t *stream )
      {
        setFromStream( output, static_cast<uint8_t>(R( stream, 255 )));
        setFromStream( output, static_cast<uint8_t>(G( stream, 255 )));
        setFromStream( output, static_cast<uint8_t>(B( stream, 255 )));
      } );
      return true;
    }
    case QVideoFrame::Format_BGR24:
    {
      auto output = *data;
      iterateImage( img, bytes_per_pixel, [ & ]( const uint8_t *stream )
      {
        setFromStream( output, static_cast<uint8_t>(B( stream, 255 )));
        setFromStream( output, static_cast<uint8_t>(G( stream, 255 )));
        setFromStream( output, static_cast<uint8_t>(R( stream, 255 )));
      } );
      return true;
    }
    case QVideoFrame::Format_RGB32:
    {
      auto output = reinterpret_cast<uint32_t *>(*data);
      iterateImage( img, bytes_per_pixel, [ & ]( const uint8_t *stream )
      {
        *output = (255U << 24U) | (static_cast<uint32_t>(static_cast<uint8_t>(R( stream, 255 ))) << 16U) |
                  (static_cast<uint32_t>(static_cast<uint8_t>(G( stream, 255 ))) << 8U) |
                  static_cast<uint32_t>(static_cast<uint8_t>(B( stream, 255 )));
        ++output;
      } );
      return true;
    }
    case QVideoFrame::Format_ARGB32:
    {
      auto output = reinterpret_cast<uint32_t *>(*data);
      iterateImage( img, bytes_per_pixel, [ & ]( const uint8_t *stream )
      {
        *output = (static_cast<uint32_t>(static_cast<uint8_t>(A( stream, 255 ))) << 24U) |
                  (static_cast<uint32_t>(static_cast<uint8_t>(R( stream, 255 ))) << 16U) |
                  (static_cast<uint32_t>(static_cast<uint8_t>(G( stream, 255 ))) << 8U) |
                  static_cast<uint32_t>(static_cast<uint8_t>(B( stream, 255 )));
        ++output;
      } );
      return true;
    }
    case QVideoFrame::Format_ARGB32_Premultiplied:
    {
      auto output = reinterpret_cast<uint32_t *>(*data);
      iterateImage( img, bytes_per_pixel, [ & ]( const uint8_t *stream )
      {
        T alpha = A( stream, 255 );
        *output = (static_cast<uint32_t>(static_cast<uint8_t>(alpha)) << 24U) |
                  (static_cast<uint32_t>(alpha * static_cast<uint8_t>(R( stream, 255 )) / 255) << 16U) |
                  (static_cast<uint32_t>(alpha * static_cast<uint8_t>(G( stream, 255 )) / 255) << 8U) |
                  static_cast<uint32_t>(alpha * static_cast<uint8_t>(B( stream, 255 )) / 255);
        ++output;
      } );
      return true;
    }
    case QVideoFrame::Format_BGR32:
    {
      auto output = reinterpret_cast<uint32_t *>(*data);
      iterateImage( img, bytes_per_pixel, [ & ]( const uint8_t *stream )
      {
        // Contrary to docs in Qt 5.9.5 this is actually 0xffBBGGRR not 0xBBGGRRff
        *output = (0xffU << 24U) |
                  (static_cast<uint32_t>(static_cast<uint8_t>(B( stream, 255 ))) << 16U) |
                  (static_cast<uint32_t>(static_cast<uint8_t>(G( stream, 255 ))) << 8U) |
                  (static_cast<uint32_t>(static_cast<uint8_t>(R( stream, 255 ))) << 0U);
        ++output;
      } );
      return true;
    }
    case QVideoFrame::Format_BGRA32:
    {
      auto output = reinterpret_cast<uint32_t *>(*data);
      iterateImage( img, bytes_per_pixel, [ & ]( const uint8_t *stream )
      {
        // Contrary to docs in Qt 5.9.5 this is actually 0xAABBGGRR not 0xBBGGRRAA
        *output = (static_cast<uint32_t>(static_cast<uint8_t>(A( stream, 255 )) << 24U)) |
                  (static_cast<uint32_t>(static_cast<uint8_t>(B( stream, 255 ))) << 16U) |
                  (static_cast<uint32_t>(static_cast<uint8_t>(G( stream, 255 ))) << 8U) |
                  (static_cast<uint32_t>(static_cast<uint8_t>(R( stream, 255 ))) << 0U);
        ++output;
      } );
      return true;
    }
    case QVideoFrame::Format_BGRA32_Premultiplied:
    {
      auto output = reinterpret_cast<uint32_t *>(*data);
      iterateImage( img, bytes_per_pixel, [ & ]( const uint8_t *stream )
      {
        // Contrary to docs in Qt 5.9.5 this is actually 0xAABBGGRR not 0xBBGGRRAA
        T alpha = A( stream, 255 );
        *output = (static_cast<uint32_t>(static_cast<uint8_t>( alpha )) << 24U) |
                  (static_cast<uint32_t>(alpha * static_cast<uint8_t>(B( stream, 255 )) / 255) << 16U) |
                  (static_cast<uint32_t>(alpha * static_cast<uint8_t>(G( stream, 255 )) / 255) << 8U) |
                  (static_cast<uint32_t>(alpha * static_cast<uint8_t>(R( stream, 255 )) / 255) << 0U);
        ++output;
      } );
      return true;
    }
    case QVideoFrame::Format_Y8:
    {
      auto output = *data;
      iterateImage( img, bytes_per_pixel, [ & ]( const uint8_t *stream )
      {
        *output = static_cast<uint8_t>((R( stream, 255 ) + G( stream, 255 ) + B( stream, 255 )) / 3);
        ++output;
      } );
      return true;
    }
    case QVideoFrame::Format_Y16:
    {
      auto output = reinterpret_cast<uint16_t *>(*data);
      typedef typename std::conditional<std::is_same<T, uint8_t>::value, uint8_t, uint16_t>::type MaxType;
      const MaxType max = std::numeric_limits<MaxType>::max();
      const uint16_t mult = (std::numeric_limits<uint16_t>::max() + 1) / (max + 1);
      iterateImage( img, bytes_per_pixel, [ & ]( const uint8_t *stream )
      {
        *output = static_cast<uint16_t>((R( stream, max ) + G( stream, max ) + B( stream, max )) * mult / 3);
        ++output;
      } );
      return true;
    }
    default:
      qWarning( "Tried to convert to unknown format. This should not be happen! Please open an issue on GitHub." );
  }
  return false;
}

/*!
 * The preferred formats are tried in order. They should be chosen first by their ability to represent the input data,
 * e.g., the same channels, and second by their memory usage (prefer outputs that require no copy over those that do
 * and if a copy is needed prefer formats that use less memory).
 */
template<typename T>
QVideoFrame::PixelFormat convertToClosestFormat( const sensor_msgs::Image &img, uint8_t **data,
                                                 int &num_bytes, int &bytes_per_line, int bytes_per_pixel,
                                                 const QList<QVideoFrame::PixelFormat> &native_formats,
                                                 const QList<QVideoFrame::PixelFormat> &supported_formats,
                                                 const QList<QVideoFrame::PixelFormat> &preferred_formats,
                                                 const Extractor<T> &R, const Extractor<T> &G,
                                                 const Extractor<T> &B, const Extractor<T> &A )
{
  for ( QVideoFrame::PixelFormat format : preferred_formats )
  {
    if ( supported_formats.contains( format ))
    {
      if ( native_formats.contains( format )) return format; // No conversion necessary
      if ( convertToFormat( img, format, data, num_bytes, bytes_per_line, bytes_per_pixel, R, G, B, A ))
        return format;
    }
  }
  return QVideoFrame::Format_Invalid;
}

bool isBigEndian()
{
  uint32_t value = 0x01;
  const void *ptr = &value;
  return *static_cast<const uint8_t *>(ptr) == 0;
}

template<typename T, int channel>
T extractChannel( const uint8_t *stream, T max )
{
  T val = *(reinterpret_cast<const T *>(stream) + channel);
  return val / ((std::numeric_limits<T>::max() + 1) / (max + 1));
}

template<typename T, int channel>
typename std::enable_if<sizeof( T ) == 1, T>::type extractChannelFromBigEndian( const uint8_t *stream, T max )
{
  T val = *(reinterpret_cast<const T *>(stream) + channel);
  return val / ((std::numeric_limits<T>::max() + 1) / (max + 1));
}

template<typename T, int channel>
typename std::enable_if<sizeof( T ) == 2, T>::type extractChannelFromBigEndian( const uint8_t *stream, T max )
{
  uint16_t tmp = (static_cast<uint32_t>(*(stream + 2 * channel)) << 8U) | *(stream + 2 * channel + 1);
  T val = *reinterpret_cast<const T *>(&tmp);
  return val / ((std::numeric_limits<T>::max() + 1) / (max + 1));
}

template<typename T, int channel>
typename std::enable_if<sizeof( T ) == 4, T>::type extractChannelFromBigEndian( const uint8_t *stream, T max )
{
  uint32_t tmp = (static_cast<uint32_t>(*(stream + 4 * channel)) << 24U) |
                 (static_cast<uint32_t>(*(stream + 4 * channel + 1)) << 16U) |
                 (static_cast<uint32_t>(*(stream + 4 * channel + 2)) << 8U) |
                 *(stream + 4 * channel + 3);
  T val = *reinterpret_cast<const T *>(&tmp);
  return val / ((std::numeric_limits<T>::max() + 1) / (max + 1));
}

template<typename T, int channel>
typename std::enable_if<sizeof( T ) == 1, T>::type extractChannelFromLittleEndian( const uint8_t *stream, T max )
{
  T val = *(reinterpret_cast<const T *>(stream) + channel);
  return val / ((std::numeric_limits<T>::max() + 1) / (max + 1));
}

template<typename T, int channel>
typename std::enable_if<sizeof( T ) == 2, T>::type extractChannelFromLittleEndian( const uint8_t *stream, T max )
{
  uint16_t tmp = (static_cast<uint32_t>(*(stream + 2 * channel + 1)) << 8U) | *(stream + 2 * channel);
  T val = *reinterpret_cast<const T *>(&tmp);
  return val / ((std::numeric_limits<T>::max() + 1) / (max + 1));
}

template<typename T, int channel>
typename std::enable_if<sizeof( T ) == 4, T>::type extractChannelFromLittleEndian( const uint8_t *stream, T max )
{
  uint32_t tmp = (static_cast<uint32_t>(*(stream + 4 * channel + 3)) << 24U) |
                 (static_cast<uint32_t>(*(stream + 4 * channel + 2)) << 16U) |
                 (static_cast<uint32_t>(*(stream + 4 * channel + 1)) << 8U) |
                 *(stream + 4 * channel);
  T val = *reinterpret_cast<const T *>(&tmp);
  return val / ((std::numeric_limits<T>::max() + 1) / (max + 1));
}


QVideoFrame::PixelFormat convertFrame( const sensor_msgs::Image &img, uint8_t **data, int &num_bytes,
                                       int &bytes_per_line, const QList<QVideoFrame::PixelFormat> &supported_formats )
{
  num_bytes = img.data.size();
  bytes_per_line = img.step;
  bool same_endianness = static_cast<bool>(img.is_bigendian) == isBigEndian();
  if ( img.encoding == sensor_msgs::image_encodings::RGB8 )
  {
    return convertToClosestFormat<uint8_t>( img, data, num_bytes, bytes_per_line, 3,
                                            { QVideoFrame::Format_RGB24 }, supported_formats,
                                            { QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                              QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
                                              QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                              QVideoFrame::Format_ARGB32_Premultiplied,
                                              QVideoFrame::Format_BGRA32_Premultiplied,
                                              QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 },
                                            extractChannel<uint8_t, 0>, extractChannel<uint8_t, 1>,
                                            extractChannel<uint8_t, 2>,
                                            []( const uint8_t *, uint8_t ) { return 255; } );
  }
  else if ( img.encoding == sensor_msgs::image_encodings::BGR8 )
  {
    return convertToClosestFormat<uint8_t>( img, data, num_bytes, bytes_per_line, 3,
                                            { QVideoFrame::Format_BGR24 }, supported_formats,
                                            { QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB24,
                                              QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB32,
                                              QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
                                              QVideoFrame::Format_BGRA32_Premultiplied,
                                              QVideoFrame::Format_ARGB32_Premultiplied,
                                              QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 },
                                            extractChannel<uint8_t, 2>, extractChannel<uint8_t, 1>,
                                            extractChannel<uint8_t, 0>,
                                            []( const uint8_t *, uint8_t ) { return 255; } );
  }
  else if ( img.encoding == sensor_msgs::image_encodings::MONO8 ||
            img.encoding == sensor_msgs::image_encodings::TYPE_8UC1 )
  {
    return convertToClosestFormat<uint8_t>( img, data, num_bytes, bytes_per_line, 1,
                                            { QVideoFrame::Format_Y8 }, supported_formats,
                                            { QVideoFrame::Format_Y8, QVideoFrame::Format_Y16,
                                              QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                              QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
                                              QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                              QVideoFrame::Format_ARGB32_Premultiplied,
                                              QVideoFrame::Format_BGRA32_Premultiplied },
                                            extractChannel<uint8_t, 0>, extractChannel<uint8_t, 0>,
                                            extractChannel<uint8_t, 0>,
                                            []( const uint8_t *, uint8_t ) { return 255; } );
  }
  else if ( img.encoding == sensor_msgs::image_encodings::MONO16 ||
            img.encoding == sensor_msgs::image_encodings::TYPE_16UC1 )
  {
    Extractor<uint16_t> extractor;
    if ( same_endianness )
      extractor = extractChannel<uint16_t, 0>;
    else if ( img.is_bigendian )
      extractor = extractChannelFromBigEndian<uint16_t, 0>;
    else
      extractor = extractChannelFromLittleEndian<uint16_t, 0>;
    QList<QVideoFrame::PixelFormat> native_formats;
    if ( same_endianness ) native_formats = { QVideoFrame::Format_Y16 };
    return convertToClosestFormat<uint16_t>( img, data, num_bytes, bytes_per_line, 2,
                                             native_formats, supported_formats,
                                             { QVideoFrame::Format_Y16, QVideoFrame::Format_Y8,
                                               QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                               QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
                                               QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                               QVideoFrame::Format_ARGB32_Premultiplied,
                                               QVideoFrame::Format_BGRA32_Premultiplied },
                                             extractor, extractor, extractor,
                                             []( const uint8_t *, uint16_t max ) { return max; } );
  }
  else if ( img.encoding == sensor_msgs::image_encodings::RGB16 )
  {
    Extractor<uint16_t> R, G, B;
    if ( same_endianness )
    {
      R = extractChannel<uint16_t, 0>;
      G = extractChannel<uint16_t, 1>;
      B = extractChannel<uint16_t, 2>;
    }
    else if ( img.is_bigendian )
    {
      R = extractChannelFromBigEndian<uint16_t, 0>;
      G = extractChannelFromBigEndian<uint16_t, 1>;
      B = extractChannelFromBigEndian<uint16_t, 2>;
    }
    else
    {
      R = extractChannelFromLittleEndian<uint16_t, 0>;
      G = extractChannelFromLittleEndian<uint16_t, 1>;
      B = extractChannelFromLittleEndian<uint16_t, 2>;
    }
    return convertToClosestFormat<uint16_t>( img, data, num_bytes, bytes_per_line, 6,
                                             {}, supported_formats,
                                             { QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                               QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
                                               QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                               QVideoFrame::Format_ARGB32_Premultiplied,
                                               QVideoFrame::Format_BGRA32_Premultiplied,
                                               QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 },
                                             R, G, B, []( const uint8_t *, uint16_t max ) { return max; } );
  }
  else if ( img.encoding == sensor_msgs::image_encodings::BGR16 )
  {
    Extractor<uint16_t> R, G, B;
    if ( same_endianness )
    {
      B = extractChannel<uint16_t, 0>;
      G = extractChannel<uint16_t, 1>;
      R = extractChannel<uint16_t, 2>;
    }
    else if ( img.is_bigendian )
    {
      B = extractChannelFromBigEndian<uint16_t, 0>;
      G = extractChannelFromBigEndian<uint16_t, 1>;
      R = extractChannelFromBigEndian<uint16_t, 2>;
    }
    else
    {
      B = extractChannelFromLittleEndian<uint16_t, 0>;
      G = extractChannelFromLittleEndian<uint16_t, 1>;
      R = extractChannelFromLittleEndian<uint16_t, 2>;
    }
    return convertToClosestFormat<uint16_t>( img, data, num_bytes, bytes_per_line, 6,
                                             {}, supported_formats,
                                             { QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB24,
                                               QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB32,
                                               QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
                                               QVideoFrame::Format_BGRA32_Premultiplied,
                                               QVideoFrame::Format_ARGB32_Premultiplied,
                                               QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 },
                                             R, G, B, []( const uint8_t *, uint16_t max ) { return max; } );
  }
  else if ( img.encoding == sensor_msgs::image_encodings::RGBA8 )
  {
    // Native format is BGRA32 if little endian because then RGBA in memory is ABGR as an uint32 which is somehow
    // the actual layout of BGRA32 in Qt 5.9.5 (contrary to what the documentation says)
    QList<QVideoFrame::PixelFormat> native_formats;
    if ( !isBigEndian()) native_formats = { QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGR32 };
    return convertToClosestFormat<uint8_t>( img, data, num_bytes, bytes_per_line, 4,
                                            native_formats, supported_formats,
                                            { QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
                                              QVideoFrame::Format_ARGB32_Premultiplied,
                                              QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_BGR32,
                                              QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                              QVideoFrame::Format_RGB32,
                                              QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 },
                                            extractChannel<uint8_t, 0>, extractChannel<uint8_t, 1>,
                                            extractChannel<uint8_t, 2>, extractChannel<uint8_t, 3> );
  }
  else if ( img.encoding == sensor_msgs::image_encodings::BGRA8 )
  {
    // Native format is ARGB32 if little endian because then BGRA in memory is ARGB as an uint32
    QList<QVideoFrame::PixelFormat> native_formats;
    if ( !isBigEndian()) native_formats = { QVideoFrame::Format_ARGB32, QVideoFrame::Format_RGB32 };
    return convertToClosestFormat<uint8_t>( img, data, num_bytes, bytes_per_line, 4,
                                            native_formats,
                                            supported_formats,
                                            { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                              QVideoFrame::Format_BGRA32_Premultiplied,
                                              QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB32,
                                              QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB24,
                                              QVideoFrame::Format_BGR32,
                                              QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 },
                                            extractChannel<uint8_t, 2>, extractChannel<uint8_t, 1>,
                                            extractChannel<uint8_t, 0>, extractChannel<uint8_t, 3> );
  }
  else if ( img.encoding == sensor_msgs::image_encodings::RGBA16 )
  {
    Extractor<uint16_t> R, G, B, A;
    if ( same_endianness )
    {
      R = extractChannel<uint16_t, 0>;
      G = extractChannel<uint16_t, 1>;
      B = extractChannel<uint16_t, 2>;
      A = extractChannel<uint16_t, 3>;
    }
    else if ( img.is_bigendian )
    {
      R = extractChannelFromBigEndian<uint16_t, 0>;
      G = extractChannelFromBigEndian<uint16_t, 1>;
      B = extractChannelFromBigEndian<uint16_t, 2>;
      A = extractChannelFromBigEndian<uint16_t, 3>;
    }
    else
    {
      R = extractChannelFromLittleEndian<uint16_t, 0>;
      G = extractChannelFromLittleEndian<uint16_t, 1>;
      B = extractChannelFromLittleEndian<uint16_t, 2>;
      A = extractChannelFromLittleEndian<uint16_t, 3>;
    }
    return convertToClosestFormat<uint16_t>( img, data, num_bytes, bytes_per_line, 8,
                                             {}, supported_formats,
                                             { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                               QVideoFrame::Format_ARGB32_Premultiplied,
                                               QVideoFrame::Format_BGRA32_Premultiplied,
                                               QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                               QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
                                               QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 },
                                             R, G, B, A );
  }
  else if ( img.encoding == sensor_msgs::image_encodings::BGRA16 )
  {
    Extractor<uint16_t> R, G, B, A;
    if ( same_endianness )
    {
      B = extractChannel<uint16_t, 0>;
      G = extractChannel<uint16_t, 1>;
      R = extractChannel<uint16_t, 2>;
      A = extractChannel<uint16_t, 3>;
    }
    else if ( img.is_bigendian )
    {
      B = extractChannelFromBigEndian<uint16_t, 0>;
      G = extractChannelFromBigEndian<uint16_t, 1>;
      R = extractChannelFromBigEndian<uint16_t, 2>;
      A = extractChannelFromBigEndian<uint16_t, 3>;
    }
    else
    {
      B = extractChannelFromLittleEndian<uint16_t, 0>;
      G = extractChannelFromLittleEndian<uint16_t, 1>;
      R = extractChannelFromLittleEndian<uint16_t, 2>;
      A = extractChannelFromLittleEndian<uint16_t, 3>;
    }
    return convertToClosestFormat<uint16_t>( img, data, num_bytes, bytes_per_line, 8,
                                             {}, supported_formats,
                                             { QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
                                               QVideoFrame::Format_BGRA32_Premultiplied,
                                               QVideoFrame::Format_ARGB32_Premultiplied,
                                               QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB24,
                                               QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB32,
                                               QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 },
                                             R, G, B, A );
  }
  else if ( img.encoding == sensor_msgs::image_encodings::TYPE_32FC1 )
  {
    Extractor<float> extractor;
    if ( img.is_bigendian )
      extractor = []( const uint8_t *p, float max ) -> float
      {
        uint32_t tmp = (static_cast<uint32_t>(*p) << 24U) | (static_cast<uint32_t>(*(p + 1)) << 16U) |
                       (static_cast<uint32_t>(*(p + 2)) << 8U) | (static_cast<uint32_t>(*(p + 3)));
        return *reinterpret_cast<float *>(&tmp) * max;
      };
    else
      extractor = []( const uint8_t *p, float max ) -> float
      {
        uint32_t tmp = (static_cast<uint32_t>(*(p + 3)) << 24U) | (static_cast<uint32_t>(*(p + 2)) << 16U) |
                       (static_cast<uint32_t>(*(p + 1)) << 8U) | (static_cast<uint32_t>(*p));
        return *reinterpret_cast<float *>(&tmp) * max;
      };
    return convertToClosestFormat<float>( img, data, num_bytes, bytes_per_line, 4,
                                          {}, supported_formats,
                                          { QVideoFrame::Format_Y16, QVideoFrame::Format_Y8,
                                            QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                            QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
                                            QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                            QVideoFrame::Format_ARGB32_Premultiplied,
                                            QVideoFrame::Format_BGRA32_Premultiplied },
                                          extractor, extractor, extractor,
                                          []( const uint8_t *, float max ) { return max; } );
  }
  return QVideoFrame::Format_Invalid;
}
}

ImageBuffer::ImageBuffer( sensor_msgs::ImageConstPtr img, const QList<QVideoFrame::PixelFormat> &supported_formats )
  : QAbstractVideoBuffer( QAbstractVideoBuffer::NoHandle ), image_( std::move( img )), data_( nullptr )
{
  format_ = convertFrame( *image_, &data_, num_bytes_, bytes_per_line_, supported_formats );
}

ImageBuffer::~ImageBuffer() { delete[] data_; }

QAbstractVideoBuffer::MapMode ImageBuffer::mapMode() const
{
  return ReadOnly;
}

uchar *ImageBuffer::map( QAbstractVideoBuffer::MapMode, int *num_bytes, int *bytes_per_line )
{
  if ( num_bytes != nullptr )
    *num_bytes = num_bytes_;
  if ( bytes_per_line != nullptr )
    *bytes_per_line = bytes_per_line_;
  if ( data_ != nullptr ) return data_;
  return const_cast<uchar *>(image_->data.data());
}

void ImageBuffer::unmap() { }

QVideoFrame::PixelFormat ImageBuffer::format() const { return format_; }
}
