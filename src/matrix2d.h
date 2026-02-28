/* matrix2d.h - Легковесная библиотека для матриц 2x3 (6 float) с накоплением */
#pragma once

#include <math.h>

typedef struct {
  float m[6];  // [a, b, c, d, e, f]
               // x' = a*x + b*y + c
               // y' = d*x + e*y + f
} Mat2D;

// Инициализация единичной матрицей
static inline void mat2d_identity(Mat2D* out) {
  out->m[0] = 1.0f;
  out->m[1] = 0.0f;
  out->m[2] = 0.0f;
  out->m[3] = 0.0f;
  out->m[4] = 1.0f;
  out->m[5] = 0.0f;
}

// Умножение матриц: res = A * B (совместимо с форматом LGFX)
static inline void mat2d_multiply(Mat2D* res, const Mat2D* A, const Mat2D* B) {
  float a0 = A->m[0], a1 = A->m[1], a2 = A->m[2];
  float a3 = A->m[3], a4 = A->m[4], a5 = A->m[5];

  float b0 = B->m[0], b1 = B->m[1], b2 = B->m[2];
  float b3 = B->m[3], b4 = B->m[4], b5 = B->m[5];

  // Row 1
  res->m[0] = a0 * b0 + a1 * b3;
  res->m[1] = a0 * b1 + a1 * b4;
  res->m[2] = a0 * b2 + a1 * b5 + a2;

  // Row 2
  res->m[3] = a3 * b0 + a4 * b3;
  res->m[4] = a3 * b1 + a4 * b4;
  res->m[5] = a3 * b2 + a4 * b5 + a5;
}

// Применение трансформации к точке
static inline void mat2d_transform_point(const Mat2D* M, float x, float y, float* ox, float* oy) {
  *ox = M->m[0] * x + M->m[1] * y + M->m[2];
  *oy = M->m[3] * x + M->m[4] * y + M->m[5];
}

// Вращение поверх текущей матрицы: res = res * Rotate
static inline void mat2d_rotate(Mat2D* res, float rad) {
  float c = cosf(rad);
  float s = sinf(rad);

  // Матрица вращения в формате LGFX
  // | c  -s  0 |
  // | s   c  0 |
  Mat2D R = {{c, -s, 0.0f, s, c, 0.0f}};

  Mat2D old = *res;
  mat2d_multiply(res, &old, &R);
}

// Перемещение поверх текущей матрицы: res = res * Translate
static inline void mat2d_translate(Mat2D* res, float tx, float ty) {
  // Матрица перемещения в формате LGFX
  // | 1  0  tx |
  // | 0  1  ty |
  Mat2D T = {{1.0f, 0.0f, tx, 0.0f, 1.0f, ty}};

  Mat2D old = *res;
  mat2d_multiply(res, &old, &T);
}

// Масштабирование поверх текущей матрицы: res = res * Scale
static inline void mat2d_scale(Mat2D* res, float sx, float sy) {
  // Матрица масштабирования в формате LGFX
  // | sx  0   0 |
  // | 0   sy  0 |
  Mat2D S = {{sx, 0.0f, 0.0f, 0.0f, sy, 0.0f}};

  Mat2D old = *res;
  mat2d_multiply(res, &old, &S);
}
