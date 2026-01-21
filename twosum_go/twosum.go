package main

import (
	"fmt"
)

// TwoSum 找出数组中两个数的下标，使其和等于目标值
// 参数：
//
//	nums: 输入的整数数组
//	target: 目标和值
//
// 返回值：
//
//	[]int: 符合条件的两个数的下标（顺序不限）
//
// 注意：题目保证输入有且仅有一个答案，无需处理无结果的情况
func TwoSum(nums []int, target int) []int {
	// 定义map，键：数组元素值，值：元素对应的下标
	numMap := make(map[int]int)

	// 遍历数组，同时获取下标和元素值
	for index, num := range nums {
		// 计算当前元素需要的「补数」（target - 当前值）
		complement := target - num
		// 检查补数是否已存在于map中（即是否遍历过该补数）
		if prevIndex, exists := numMap[complement]; exists {
			// 存在则返回补数的下标和当前元素的下标
			return []int{prevIndex, index}
		}
		// 不存在则将当前元素和下标存入map（先查后存，避免使用同一元素）
		numMap[num] = index
	}

	// 题目保证有答案，此处仅为语法完整性
	return nil
}

func main() {
	// 测试用例1：基础场景
	nums1 := []int{2, 7, 11, 15}
	target1 := 9
	fmt.Printf("测试用例1结果：%v\n", TwoSum(nums1, target1)) // 输出 [0 1]

	// 测试用例2：非连续元素
	nums2 := []int{3, 2, 4}
	target2 := 6
	fmt.Printf("测试用例2结果：%v\n", TwoSum(nums2, target2)) // 输出 [1 2]

	// 测试用例3：重复元素
	nums3 := []int{3, 3}
	target3 := 6
	fmt.Printf("测试用例3结果：%v\n", TwoSum(nums3, target3)) // 输出 [0 1]
}
