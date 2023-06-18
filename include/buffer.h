#ifndef BUFFER_H
#define BUFFER_H
#include <cstring>
#include <iostream>

class Buffer {
private:
  static int const MAX_SIZE = 200;
  char *data_;
  int size_;
  int count_;
  int read_from_{0};
  int write_from_{0};

public:
  Buffer(int size, int count) {
    size_ = size;
    count_ = count;
    data_ = (char *)std::malloc(size * count);
  }

  void write(char *data, int size) {
    memcpy(data_ + write_from_, data, size);
    write_from_ += size;
  }

  void read(char *data, int size) {
    memcpy(data, data_ + read_from_, size);
    read_from_ += size;
  }
};

#endif
