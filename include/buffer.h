#ifndef BUFFER_H
#define BUFFER_H
template <typename T> class Buffer {
public:
  T data;
  Buffer(T _data) { data = _data; }
};

#endif
