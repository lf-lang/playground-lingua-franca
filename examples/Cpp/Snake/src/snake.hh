#pragma once

#include <vector>

enum class Direction {
  UP = -1,
  DOWN = 1,
  RIGHT = -2,
  LEFT = 2
};

enum class BoardElement {
  EMPTY,
  SNAKE_BODY,
  SNAKE_HEAD,
  APPLE,
  WALL,
  INVALID,
};

class Board {
 private:
  unsigned size;
  std::vector<BoardElement> elements;

 public:
  Board(unsigned size)
    : size{size}
    , elements{size * size, BoardElement::EMPTY} {}

  auto get_size() const { return size; }
    
  auto is_valid(int x, int y) const -> bool { 
    return x >= 0 && y >= 0 && x < static_cast<int>(size) && y < static_cast<int>(size); 
  }

  auto at(int x, int y) const -> BoardElement { 
    if (is_valid(x, y)) {
      return elements[y * size + x];
    }
    return BoardElement::INVALID;
  }

  void set(int x, int y, BoardElement element) {
    if (is_valid(x, y)) {
      elements[y * size + x] = element;
    }
  }
};