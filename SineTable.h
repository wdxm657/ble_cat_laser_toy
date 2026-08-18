#ifndef _SineTable_h_
#define _SineTable_h_

/* constants */
#define TABLE_Q15_SIZE  257       // 0..256: sin(0) ~ sin(pi/2)
#define SIN_TABLE_SCALE 32767.0f  // Q15: 1.0 → 32767

/* Q15 正弦查找表 (int16)，覆盖 [0, pi/2] */
extern const signed short sinetable_q15[TABLE_Q15_SIZE];

/* public f(x) — 返回值仍为 float 以兼容调用方 */
extern float lookup_sin (float x);
extern float lookup_cos (float x);
extern float lookup_tan (float x);
extern float lookup_cot (float x);
extern float lookup_atan2(float y, float x);

#endif	// _SineTable_h_
