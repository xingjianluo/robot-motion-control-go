GO ?= go

.PHONY: all build test demo verbose vet fmt check clean

all: build

build:
	$(GO) build ./...

test:
	$(GO) test ./... -race -count=1

demo:
	$(GO) run ./cmd/demo

verbose:
	$(GO) test ./internal/... -race -count=1 -v

vet:
	$(GO) vet ./...

fmt:
	$(GO) fmt ./...

check: vet test

clean:
	$(GO) clean -testcache
