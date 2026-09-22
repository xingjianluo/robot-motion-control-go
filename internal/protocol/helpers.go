package protocol

import "strconv"

func formatFloat(v float64) string { return strconv.FormatFloat(v, 'f', -1, 64) }
func itoa(v int) string            { return strconv.Itoa(v) }
func boolStr(b bool) string {
	if b {
		return "1"
	}
	return "0"
}
