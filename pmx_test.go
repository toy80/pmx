package pmx

import (
	"os"
	"testing"
)

func TestDecode(t *testing.T) {
	f, err := os.Open("testdata/testing.pmx")
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	p, err := Decode(f)
	if err != nil {
		t.Fatal(err)
	}
	if p.Header.Version != 2.1 {
		t.Fatal(err)
	}
}
