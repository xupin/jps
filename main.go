package main

import (
	"fmt"
	"time"

	"github.com/xupin/jps/jps"
)

func main() {
	j := &jps.Jps{
		Rows:      5,
		Cols:      8,
		Heuristic: jps.Diagonal,
	}
	// 5x8地图
	// 0是可移动的网格
	// 1是障碍网格
	mapData := [][]int{
		{0, 0, 0, 0, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 1, 0, 0},
		{0, 0, 0, 0, 0, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
	}
	j.Init(mapData)
	fmt.Println("开始时间", time.Now().UnixNano())
	node := j.FindPath(
		&jps.Node{X: 0, Y: 0},
		&jps.Node{X: 6, Y: 2},
	)
	fmt.Println("结束时间", time.Now().UnixNano())
	j.Print(node, mapData)
}
