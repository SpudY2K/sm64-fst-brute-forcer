kernel void test(const global uint* vals, global uint* squares) {
  uint id = get_global_id(0);
  squares[id] = vals[id] * vals[id];
}