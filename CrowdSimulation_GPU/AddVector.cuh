#include "cuda_runtime.h"
#include "box2d.h"

void addVectorI(float32 *re, float32 *a, float32 *b, int size);
void addVectorC(float32 *c, float32 k, int size);
void addVectorV(b2Vec2 *c, b2Vec2 vec, int size);