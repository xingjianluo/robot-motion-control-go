package feedback

import "strconv"

func u32(n uint32) string  { return strconv.FormatUint(uint64(n), 10) }
func i64(n int64) string   { return strconv.FormatInt(n, 10) }
func f64(f float64) string { return strconv.FormatFloat(f, 'f', 4, 64) }
