package jps

import (
	"fmt"
	"math"
	"sort"
)

/*
地图从左上角开始，水平x 垂直y
  y
x 0,0 1,0 2,0
  0,1 1,1 2,1
  0,2 1,2 2,2
*/

// 节点
type Node struct {
	// 坐标
	X int
	Y int
	// 移动成本
	F int
	G int
	H int
	// 父节点
	Parent *Node
	// 类型
	Type int
	// 状态
	State int
}

type Jps struct {
	// 启发算法
	Heuristic func(node, end *Node) int
	// 地图大小
	Rows int // y
	Cols int // x
	// 地图节点
	nodes [][]*Node
	start *Node
	end   *Node
	// 开放、关闭列表
	openList  []*Node
	closeList []*Node
	// 对角相邻坐标
	neighborPos [][]int
}

// 移动成本
const (
	COST_STRAIGHT = 10
	COST_DIAGONAL = 14
)

// 节点类型
const (
	NODE_TYPE_NORMAL = iota
	NODE_TYPE_OBSTACLE
)

// 节点状态
const (
	NODE_STATE_CLOSED = iota - 1
	NODE_STATE_NORMAL
	NODE_STATE_OPENED
)

func (r *Jps) Init(mapData [][]int) {
	r.nodes = make([][]*Node, r.Cols)
	for i := 0; i < r.Cols; i++ {
		r.nodes[i] = make([]*Node, r.Rows)
	}
	for i := 0; i < len(mapData); i++ {
		for j := 0; j < len(mapData[i]); j++ {
			node := &Node{
				X:    j,
				Y:    i,
				Type: mapData[i][j],
			}
			r.nodes[j][i] = node
		}
	}
	// 如果不允许对角移动，去除对角坐标
	r.neighborPos = [][]int{
		{0, -1},  // 上
		{1, -1},  // 右上
		{1, 0},   // 右
		{1, 1},   // 右下
		{0, 1},   // 下
		{-1, 1},  // 左下
		{-1, 0},  // 左
		{-1, -1}, // 左上
	}
}

func (r *Jps) FindPath(start, end *Node) *Node {
	r.start = r.getNode(start.X, start.Y)
	r.end = r.getNode(end.X, end.Y)
	// 如果起止点是障碍物
	if !r.start.isWalkable() || !r.end.isWalkable() {
		fmt.Println("障碍物不可移动")
		return nil
	}
	// 先把开始节点放进开放列表
	r.openListAppend(r.start)
	for len(r.openList) > 0 {
		node := r.openListPop()
		// 判断当前节点是否是终点
		if r.isEnd(node) {
			return node
		}
		// 找开放列表的第一个节点的相邻节点
		neighbors := r.findNeighbors(node)
		for _, neighbor := range neighbors {
			jump := r.jump(neighbor, node)
			// 无跳点 || 节点已被关闭
			if jump == nil || jump.isClosed() {
				continue
			}
			// 当前节点的成本
			// 跳点的x,y
			g, x, y := node.G, jump.X, jump.Y
			// 判断移动方式是水平（或垂直）、对角，计算成本
			if x == node.X {
				g += abs(y-node.Y) * COST_STRAIGHT
			} else if y == node.Y {
				g += abs(x-node.X) * COST_STRAIGHT
			} else {
				g += abs(x-node.X) * COST_DIAGONAL
			}
			if !jump.isOpened() || g < jump.G {
				jump.G = g
				jump.H = r.Heuristic(jump, r.end)
				jump.F = jump.G + jump.H
				jump.Parent = node
				// 优化逻辑，跳点是否是终点
				if r.isEnd(jump) {
					return jump
				}
				if !jump.isOpened() {
					r.openListAppend(jump)
				}
			}
		}
		// 当前节点放进关闭列表
		r.closeListAppend(node)
		// 更新开放列表顺序
		r.openListSort()
	}
	return nil
}

// 跳点函数
// 判断当前点是否满足跳点条件
func (r *Jps) jump(node, parent *Node) *Node {
	// 是终点，直接返回
	if r.isEnd(node) {
		return node
	}
	x, y := node.X, node.Y
	dx, dy := r.direction(node, parent)
	// 对角移动
	if dx != 0 && dy != 0 {
		// [左|右]不能走 && [左上|左下|右上|右下]能走
		if !r.isWalkable(x-dx, y) && r.isWalkable(x-dx, y+dy) {
			return node
		}
		// [上|下]不能走 && [左上|右上|左下|右下]能走
		if !r.isWalkable(x, y-dy) && r.isWalkable(x+dx, y-dy) {
			return node
		}
		// 递归查找方向[上|下]继续查找
		if r.isWalkable(x+dx, y) && r.jump(r.nodes[x+dx][y], node) != nil {
			return node
		}
		// 递归查找方向[左|右]继续查找
		if r.isWalkable(x, y+dy) && r.jump(r.nodes[x][y+dy], node) != nil {
			return node
		}
	} else if dx == 0 { // 垂直移动
		// 右不能走 && [右下|右上]能走
		if !r.isWalkable(x+1, y) && r.isWalkable(x+1, y+dy) {
			return node
		}
		// 左不能走 && [左上|左下]能走
		if !r.isWalkable(x-1, y) && r.isWalkable(x-1, y+dy) {
			return node
		}
	} else { // 水平移动
		// 下不能走 && [左下|右下]能走
		if !r.isWalkable(x, y+1) && r.isWalkable(x+dx, y+1) {
			return node
		}
		// 上不能走 && [左上|右上]能走
		if !r.isWalkable(x, y-1) && r.isWalkable(x+dx, y-1) {
			return node
		}
	}
	// 递归查找方向[左上|左下|右上|右下]继续查找
	if r.isWalkable(x+dx, y+dy) {
		if next := r.jump(r.nodes[x+dx][y+dy], node); next != nil {
			return next
		}
	}
	// 无强迫邻居（当前节点不是跳点）或到达边界
	return nil
}

// 查找相邻节点位置
func (r *Jps) findNeighbors(node *Node) []*Node {
	neighbors := make([]*Node, 0)
	// 第一次移动
	if node.Parent == nil {
		for _, v := range r.neighborPos {
			x, y := node.X+v[0], node.Y+v[1]
			// 检测节点是否非法
			if !r.isWalkable(x, y) {
				continue
			}
			neighbors = append(neighbors, r.nodes[x][y])
		}
	} else {
		// 计算当前节点位于父节点的方向：水平、垂直和对角方向
		x, y := node.X, node.Y
		dx, dy := r.direction(node, node.Parent)
		// 移动方向上的下一个
		if r.isWalkable(x+dx, y+dy) {
			neighbors = append(neighbors, r.nodes[x+dx][y+dy])
		}
		// 对角移动
		if dx != 0 && dy != 0 {
			// [左|右]能走
			if r.isWalkable(x+dx, y) {
				neighbors = append(neighbors, r.nodes[x+dx][y])
			}
			// [上|下]能走
			if r.isWalkable(x, y+dy) {
				neighbors = append(neighbors, r.nodes[x][y+dy])
			}
			// [左|右]不能走 && [左上|左下|右上|右下]能走
			if !r.isWalkable(x-dx, y) && r.isWalkable(x-dx, y+dy) {
				neighbors = append(neighbors, r.nodes[x-dx][y+dy])
			}
			// [上|下]不能走 && [左上|右上|左下|右下]能走
			if !r.isWalkable(x, y-dy) && r.isWalkable(x+dx, y-dy) {
				neighbors = append(neighbors, r.nodes[x+dx][y-dy])
			}
		} else if dx == 0 { // 垂直移动
			// 右不能走 && [右下|右上]能走
			if !r.isWalkable(x+1, y) && r.isWalkable(x+1, y+dy) {
				neighbors = append(neighbors, r.nodes[x+1][y+dy])
			}
			// 左不能走 && [左上|左下]能走
			if !r.isWalkable(x-1, y) && r.isWalkable(x-1, y+dy) {
				neighbors = append(neighbors, r.nodes[x-1][y+dy])
			}
		} else { // 水平移动
			// 下不能走 && [左下|右下]能走
			if !r.isWalkable(x, y+1) && r.isWalkable(x+dx, y+1) {
				neighbors = append(neighbors, r.nodes[x+dx][y+1])
			}
			// 上不能走 && [左上|右上]能走
			if !r.isWalkable(x, y-1) && r.isWalkable(x+dx, y-1) {
				neighbors = append(neighbors, r.nodes[x+dx][y-1])
			}
		}
	}
	return neighbors
}

func (r *Jps) isWalkable(x, y int) bool {
	// 最小越界
	if x < 0 || y < 0 {
		return false
	}
	// 最大越界
	if x > r.Cols-1 || y > r.Rows-1 {
		return false
	}
	// 节点是否障碍物
	if !r.nodes[x][y].isWalkable() {
		return false
	}
	return true
}

func (r *Jps) getNode(x, y int) *Node {
	return r.nodes[x][y]
}

func (r *Jps) isEnd(node *Node) bool {
	return node.X == r.end.X && node.Y == r.end.Y
}

func (node *Node) isWalkable() bool {
	return node.Type != NODE_TYPE_OBSTACLE
}

func (node *Node) isOpened() bool {
	return node.State == NODE_STATE_OPENED
}

func (node *Node) isClosed() bool {
	return node.State == NODE_STATE_CLOSED
}

func (r *Jps) openListAppend(node *Node) {
	node.State = NODE_STATE_OPENED
	r.openList = append(r.openList, node)
}

func (r *Jps) openListPop() *Node {
	s := r.openList
	if len(s) == 0 {
		return nil
	}
	v := s[0]
	s[0] = nil
	s = s[1:]
	r.openList = s
	return v
}

func (r *Jps) openListSort() {
	sort.Slice(r.openList, func(i, j int) bool {
		return r.openList[i].F < r.openList[j].F
	})
}

func (r *Jps) closeListAppend(node *Node) {
	node.State = NODE_STATE_CLOSED
	r.closeList = append(r.closeList, node)
}

func (a *Jps) Print(node *Node, mapData [][]int) {
	fmt.Println("导航路径：")
	for node != nil {
		fmt.Printf("x,y: %d,%d cost: f%d h%d g%d \n", node.X, node.Y, node.F, node.H, node.G)
		a.nodes[node.X][node.Y].Type = 9
		node = node.Parent
	}
	fmt.Println("导航图：")
	for i := 0; i < len(mapData); i++ {
		for j := 0; j < len(mapData[i]); j++ {
			if a.nodes[j][i].Type == 9 {
				fmt.Print("* ")
			} else {
				fmt.Print(a.nodes[j][i].Type, " ")
			}
		}
		fmt.Print("\n")
	}
	fmt.Println("准备扫描节点：")
	for _, node := range a.openList {
		fmt.Printf("x,y: %d,%d \n", node.X, node.Y)
	}
	fmt.Println("已扫描节点：")
	for _, node := range a.closeList {
		fmt.Printf("x,y: %d,%d \n", node.X, node.Y)
	}
}

// 曼哈顿
func Manhattan(node, end *Node) int {
	x := abs(node.X - end.X)
	y := abs(node.Y - end.Y)
	return (x + y) * COST_STRAIGHT
}

// 对角线
func Diagonal(node, end *Node) int {
	x := abs(node.X - end.X)
	y := abs(node.Y - end.Y)
	min := min(x, y)
	return min*COST_DIAGONAL + abs(x-y)*COST_STRAIGHT
}

// 欧几里得
func Euclidean(node, end *Node) int {
	x := abs(node.X - end.X)
	y := abs(node.Y - end.Y)
	v := float64(x)*float64(x) + float64(y)*float64(y)
	return int(math.Sqrt(v) * COST_STRAIGHT)
}

// 45度角
func Octile(node, end *Node) int {
	x := abs(node.X - end.X)
	y := abs(node.Y - end.Y)
	min, max := min(x, y), max(x, y)
	return (COST_DIAGONAL-COST_STRAIGHT)*min + max
}

// 计算移动方向
// 该函数计算结果：0,1（垂直移动）、1,0（水平移动）、1,1 | -1,n | n,-1（对角移动）
func (r *Jps) direction(node, parent *Node) (int, int) {
	x, y := node.X, node.Y
	px, py := parent.X, parent.Y
	dx := (x - px) / max(abs(x-px), 1)
	dy := (y - py) / max(abs(y-py), 1)
	// fmt.Printf(
	// 	"计算方向 px,py: %d,%d x,y:%d,%d dx,dy:%d,%d \n",
	// 	px,
	// 	py,
	// 	x,
	// 	y,
	// 	dx,
	// 	dy,
	// )
	return dx, dy
}

func abs(n int) int {
	y := n >> 63
	return (n ^ y) - y
}

func min(a, b int) int {
	if a < b {
		return a
	} else {
		return b
	}
}

func max(a, b int) int {
	if a > b {
		return a
	} else {
		return b
	}
}
