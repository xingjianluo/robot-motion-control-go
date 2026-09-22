package safety

import "github.com/xingjianluo/robot-motion-control-go/pkg/errors"

func errReason(code, detail string) error {
	return errors.New(errors.CatSafety, code, detail)
}
