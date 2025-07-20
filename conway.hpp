#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <mutex>
#include <set>
#include <functional>
#include <cassert>

#define PRINT_TICK_RESULT 0

// 模拟数据库操作
class Cache
{
private:
	// 版本 -> 节点ID -> 点数据
	std::unordered_map<int,
		std::unordered_map<std::string,
		std::unordered_map<uint16_t, bool>>>
		cache_;

	std::mutex mutex_;

	Cache() = default;
	Cache(const Cache&) = delete;
	Cache& operator=(const Cache&) = delete;

public:
	static Cache& getInstance()
	{
		static Cache instance;
		return instance;
	}

	void save(int version, const std::string& node_id,
		const std::unordered_map<uint16_t, bool>& points)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		cache_[version][node_id] = points;
	}

	void removeVersion(int version)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		cache_.erase(version);
	}

	const std::unordered_map<uint16_t, bool>* getData(int version, const std::string& node_id)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		auto ver_it = cache_.find(version);
		if (ver_it == cache_.end())
			return nullptr;

		auto node_it = ver_it->second.find(node_id);
		if (node_it == ver_it->second.end())
			return nullptr;

		return &(node_it->second);
	}
};

class Node
{
private:
	enum NodeType
	{
		INTERNAL,
		LEAF
	};
	NodeType type_;
	int depth_;                                     // 1 to 7
	int ver_;                                       // 节点版本
	std::unordered_map<uint16_t, Node*>* children;  // 内部节点使用
	std::unordered_map<uint16_t, bool>* points;     // 叶子节点使用
	std::string id_;                                // 节点唯一标识
	Node* root_;                                    // 指向根节点的指针

	static constexpr int64_t HIGH_MASK = 0xFFFFFFFFFFFFFF; // 高56位掩码

public:
	Node(int depth, Node* root = nullptr, const std::string& parent_id = "", uint16_t index = 0, int ver = 0)
		: depth_(depth), ver_(ver), children(nullptr), points(nullptr), root_(root)
	{
		if (root_ == nullptr)
		{
			root_ = this;
			id_ = parent_id;
		}
		else
		{
			uint8_t x_index = ((index >> 8) & 0xFF);
			uint8_t y_index = (index & 0xFF);

			// 构建当前节点的id
			id_ = parent_id;
			id_ += static_cast<char>(x_index); // 高字节
			id_ += static_cast<char>(y_index); // 低字节
		}

		if (depth < 8)
		{
			type_ = INTERNAL;
			children = new std::unordered_map<uint16_t, Node*>();
		}
		else
		{
			type_ = LEAF;
			points = new std::unordered_map<uint16_t, bool>();
		}
	}

	~Node()
	{
		if (children)
		{
			for (auto& child : *children)
			{
				delete child.second;
			}
			delete children;
		}
		if (points)
		{
			delete points;
		}
	}

	Node(const Node&) = delete;
	Node& operator=(const Node&) = delete;

	const std::string& getId() const { return id_; }
	int getVersion() const { return ver_; }

	void insert(int64_t x, int64_t y)
	{
		if (type_ == INTERNAL)
		{
			uint16_t index = calculateNodeIndex(x, y);
			auto it = children->find(index);
			if (it == children->end())
			{
				Node* child = new Node(depth_ + 1, root_, id_, index);
				it = children->insert({ index, child }).first;
			}
			it->second->insert(x, y);
		}
		else
		{
			uint16_t key = calculatePointKey(x, y);
			if (points->find(key) == points->end())
			{
				(*points)[key] = true;
				Cache::getInstance().save(ver_, id_, *points);
			}
		}
	}

	void pre_tick()
	{
		if (type_ == LEAF)
		{
			tryCreateNeighbors();
		}
		else
		{
			for (auto& child : *children)
			{
				child.second->pre_tick();
			}
		}
	}

	void tick()
	{
		if (type_ == LEAF)
		{
			conwayGameOfLife();
			ver_++;
			Cache::getInstance().save(ver_, id_, *points);
		}
		else
		{
			for (auto& child : *children)
			{
				child.second->tick();
			}
			// 所有子节点处理完成后增加版本
			ver_++;
		}
	}

	bool post_tick()
	{
		if (type_ == LEAF)
		{
			return shouldSelfDestruct();
		}
		else
		{
			std::set<uint16_t> to_remove;
			bool all_children_empty = true;

			for (auto& child : *children)
			{
				bool should_remove = child.second->post_tick();

				if (should_remove)
				{
					to_remove.insert(child.first);
				}
				else
				{
					all_children_empty = false;
				}
			}

			// 清理空节点
			for (auto key : to_remove)
			{
				delete (*children)[key];
				children->erase(key);
			}

			return all_children_empty;
		}
	}

	bool contains(int64_t x, int64_t y)
	{
		if (type_ == INTERNAL)
		{
			uint16_t index = calculateNodeIndex(x, y);
			auto it = children->find(index);
			if (it == children->end())
			{
				return false;
			}
			return it->second->contains(x, y);
		}
		else
		{
			uint16_t key = calculatePointKey(x, y);
			return points->find(key) != points->end();
		}
	}

	void PrintAll(bool just_point = false)
	{
		if (type_ == LEAF)
		{
			// 获取当前叶子节点的高56位坐标
			int64_t x_high = 0, y_high = 0;
			parseIdToCoordinates(x_high, y_high);
			if (!just_point)
			{
				std::cout << (x_high << 8) << "_" << (y_high << 8) << ":" << std::endl;
			}
			for (auto p : *points)
			{
				uint16_t key = p.first;
				int64_t x_low = (key >> 8) & 0xFF;
				int64_t y_low = key & 0xFF;
				int64_t x_global = (x_high << 8) | x_low;
				int64_t y_global = (y_high << 8) | y_low;
				std::cout << static_cast<int64_t>(x_global) << " " << static_cast<int64_t>(y_global) << std::endl;
			}
		}
		else
		{
			for (auto& child : *children)
			{
				child.second->PrintAll(just_point);
			}
		}
	}

private:
	void tryCreateNeighbors()
	{
		if (points->empty())
		{
			return;
		}

		std::vector<std::string> neighbor_ids = getNeighborLeafIds();

		for (const auto& nid : neighbor_ids)
		{
			// 从根节点创建路径
			root_->createPathToLeaf(nid);
		}
	}

	// 创建到目标叶子节点的路径
	void createPathToLeaf(const std::string& target_id)
	{
		// 如果当前节点就是目标
		if (id_ == target_id)
			return;
		assert(type_ != LEAF);

		uint8_t x_index = target_id[(depth_ - 1) * 2];
		uint8_t y_index = target_id[(depth_ - 1) * 2 + 1];
		uint16_t index = (static_cast<uint16_t>(x_index) << 8) + y_index;

		// 如果下一层节点不存在，创建它
		if (children->find(index) == children->end())
		{
			Node* child = new Node(depth_ + 1, root_, id_, index, ver_);
			(*children)[index] = child;
		}

		// 递归创建路径
		(*children)[index]->createPathToLeaf(target_id);
	}

	// 自身有节点或者相邻的节点有数据就保留，否则删除
	bool shouldSelfDestruct()
	{
		if (!points->empty())
			return false;

		std::vector<std::string> neighbor_ids = getNeighborLeafIds();

		for (const auto& nid : neighbor_ids)
		{
			// 检查缓存中是否有数据
			const auto* neighbor_data = Cache::getInstance().getData(ver_, nid);
			if (neighbor_data && !neighbor_data->empty())
			{
				return false;
			}
		}

		return true;
	}

	static std::pair<int64_t, int64_t> calcGlobalValue(const std::string& id, uint16_t key)
	{
		int64_t x_high = 0, y_high = 0;
		parseIdToCoordinates(x_high, y_high, id);
		int64_t x_low = (key >> 8) & 0xFF;
		int64_t y_low = key & 0xFF;
		int64_t x_global = (x_high << 8) | x_low;
		int64_t y_global = (y_high << 8) | y_low;
		return { x_global, y_global };
	}

	void conwayGameOfLife()
	{
		// 1. 获取当前版本的所有相邻叶子节点ID
		std::vector<std::string> neighbor_ids = getNeighborLeafIds();

		// 2. 构建全局活细胞集合
		using GlobalCoord = std::pair<int64_t, int64_t>;
		auto hash = [](const GlobalCoord& p)
			{
				return std::hash<int64_t>()(p.first) ^ std::hash<int64_t>()(p.second);
			};
		std::unordered_set<GlobalCoord, decltype(hash)> global_lives(8 * 1024, hash);

		// 获取当前叶子节点的高56位坐标
		int64_t x_high = 0, y_high = 0;
		parseIdToCoordinates(x_high, y_high);

		// 添加当前叶子节点的点
		for (const auto& p : *points)
		{
			uint16_t key = p.first;
			int64_t x_low = (key >> 8) & 0xFF;
			int64_t y_low = key & 0xFF;
			int64_t x_global = (x_high << 8) | x_low;
			int64_t y_global = (y_high << 8) | y_low;
			global_lives.insert({ x_global, y_global });
		}

		// 添加相邻叶子节点的点
		for (const auto& nid : neighbor_ids)
		{
			const auto* neighbor_points = Cache::getInstance().getData(ver_, nid);
			if (!neighbor_points)
				continue;

			int64_t nx_high = 0, ny_high = 0;
			parseIdToCoordinates(nx_high, ny_high, nid);

			for (const auto& p : *neighbor_points)
			{
				uint16_t key = p.first;
				int64_t x_low = (key >> 8) & 0xFF;
				int64_t y_low = key & 0xFF;
				int64_t x_global = (nx_high << 8) | x_low;
				int64_t y_global = (ny_high << 8) | y_low;
				global_lives.insert({ x_global, y_global });
			}
		}

		// 3. 统计当前叶子节点内每个位置的邻居数量
		std::unordered_map<uint16_t, int> neighbor_counts;

		for (const auto& p : global_lives)
		{
			int64_t x_global = p.first;
			int64_t y_global = p.second;

			for (int dx = -1; dx <= 1; dx++)
			{
				for (int dy = -1; dy <= 1; dy++)
				{
					if (dx == 0 && dy == 0)
						continue;

					int64_t nx = x_global + dx;
					int64_t ny = y_global + dy;

					// 只有相邻节点在本 node 内才被处理
					int64_t nx_high = ((nx >> 8) & HIGH_MASK);
					int64_t ny_high = ((ny >> 8) & HIGH_MASK);

					if (nx_high == x_high && ny_high == y_high)
					{
						uint8_t x_low_neighbor = nx & 0xFF;
						uint8_t y_low_neighbor = ny & 0xFF;
						uint16_t neighbor_key = (static_cast<uint16_t>(x_low_neighbor) << 8) | y_low_neighbor;
						neighbor_counts[neighbor_key]++;
					}
				}
			}
		}

		// 4. 状态计算
		std::vector<uint16_t> to_remove;
		std::vector<uint16_t> to_add;

		std::unordered_map<uint16_t, bool>* new_points = new std::unordered_map<uint16_t, bool>();
		// 检查当前节点内的点
		for (const auto& entry : neighbor_counts)
		{
			uint16_t key = entry.first;
			int count = entry.second;

			// 计算全局坐标
			int64_t x_low = (key >> 8) & 0xFF;
			int64_t y_low = key & 0xFF;
			int64_t x_global = (x_high << 8) | x_low;
			int64_t y_global = (y_high << 8) | y_low;

			bool is_alive = (points->find(key) != points->end());

			if (is_alive)
			{
				if (count == 2 || count == 3)
				{
					new_points->emplace(key, true);
				}
			}
			else
			{
				if (count == 3)
				{
					new_points->emplace(key, true);
				}
			}
		}

		delete points;
		points = new_points;
	}

	// 获取相邻格子的ID
	std::vector<std::string> getNeighborLeafIds() const
	{
		std::vector<std::string> neighbors;
		if (type_ != LEAF)
			return neighbors;

		// 解析当前叶子节点的路径
		int64_t x_high = 0, y_high = 0;
		parseIdToCoordinates(x_high, y_high);

		// 8个方向
		const int directions[8][2] = {
			{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1} };

		for (const auto& dir : directions)
		{
			int dx = dir[0];
			int dy = dir[1];

			// 相邻格子的路径
			int64_t new_x_high = x_high;
			int64_t new_y_high = y_high;

			if (dx == -1)
			{
				if (x_high == ((std::numeric_limits<int64_t>::min() >> 8) & HIGH_MASK))
					continue;
				new_x_high--;
			}
			else if (dx == 1)
			{
				if (x_high == ((std::numeric_limits<int64_t>::max() >> 8) & HIGH_MASK))
					continue;
				new_x_high++;
			}

			if (dy == -1)
			{
				if (y_high == ((std::numeric_limits<int64_t>::min() >> 8) & HIGH_MASK))
					continue;
				new_y_high--;
			}
			else if (dy == 1)
			{
				if (y_high == ((std::numeric_limits<int64_t>::max() >> 8) & HIGH_MASK))
					continue;
				new_y_high++;
			}

			neighbors.push_back(coordinatesToId(new_x_high, new_y_high));
		}

		return neighbors;
	}

	void parseIdToCoordinates(int64_t& x_high, int64_t& y_high) const
	{
		parseIdToCoordinates(x_high, y_high, id_);
	}

	// ID转坐标
	static void parseIdToCoordinates(int64_t& x_high, int64_t& y_high, const std::string& id)
	{
		x_high = 0;
		y_high = 0;
		for (size_t i = 0; i < id.size(); i += 2)
		{
			uint8_t x_byte = static_cast<uint8_t>(id[i]);
			uint8_t y_byte = static_cast<uint8_t>(id[i + 1]);
			x_high = (x_high << 8) | x_byte;
			y_high = (y_high << 8) | y_byte;
		}
	}

	uint16_t calculateNodeIndex(int64_t x, int64_t y) const
	{
		// 计算 x, y 这个坐标在这层 Node 中应该映射到哪个 index 中去 
		int shift = 64 - (depth_ * 8);
		uint8_t x_part = (x >> shift) & 0xFF;
		uint8_t y_part = (y >> shift) & 0xFF;
		return (static_cast<uint16_t>(x_part) << 8) | y_part;
	}

	uint16_t calculatePointKey(int64_t x, int64_t y) const
	{
		uint8_t x_low = x & 0xFF;
		uint8_t y_low = y & 0xFF;
		return (static_cast<uint16_t>(x_low) << 8) | y_low;
	}

	std::string coordinatesToId(int64_t x_high, int64_t y_high) const
	{
		std::string new_id;
		// 提取x_high和y_high的7个字节（56位）
		for (int i = 6; i >= 0; --i)
		{
			uint8_t x_byte = (x_high >> (i * 8)) & 0xFF;
			uint8_t y_byte = (y_high >> (i * 8)) & 0xFF;
			new_id += static_cast<char>(x_byte);
			new_id += static_cast<char>(y_byte);
		}
		return new_id;
	}
};

class ConwayGameOfLife
{
private:
	Node* root;
public:
	ConwayGameOfLife() :root(new Node(1))
	{
	}

	~ConwayGameOfLife()
	{
		if (root)
		{
			delete root;
		}
	}

	void tick(size_t num = 1)
	{
		for (size_t i = 0; i < num; i++)
		{
			root->pre_tick();
#if PRINT_TICK_RESULT
			std::cout << "pre_tick ------------------" << std::endl;
			root->PrintAll();
#endif // PRINT_TICK_RESULT
			auto old_ver = root->getVersion();
			root->tick();
			Cache::getInstance().removeVersion(old_ver);
#if PRINT_TICK_RESULT
			std::cout << "tick ------------------" << std::endl;
			root->PrintAll();
#endif // PRINT_TICK_RESULT
			root->post_tick();
#if PRINT_TICK_RESULT
			std::cout << "post_tick ------------------" << std::endl;
			root->PrintAll();
#endif // PRINT_TICK_RESULT

		}
	}

	void reset(std::string life106Str)
	{
		if (root)
		{
			delete root;
			root = new Node(1);
		}
		auto life106 = parse_life106_string(life106Str);
		for (auto& it : life106)
		{
			root->insert(it.first, it.second);
		}
	}

	void print()
	{
		if (root)
		{
			root->PrintAll(true);
		}
	}

	// 从 life106 格式的文件中加载数据
	static std::vector<std::pair<int, int>> parse_life106_string(const std::string& life_string) {
		std::vector<std::pair<int, int>> coordinates;
		std::istringstream iss(life_string);
		std::string line;

		while (std::getline(iss, line)) {
			size_t start = line.find_first_not_of(" \t");
			if (start == std::string::npos) continue;
			size_t end = line.find_last_not_of(" \t");
			std::string trimmed = line.substr(start, end - start + 1);
			if (trimmed.empty() || trimmed[0] == '#') continue;

			std::istringstream line_stream(trimmed);
			int x, y;
			if (line_stream >> x >> y) {
				coordinates.emplace_back(x, y);
			}
		}

		return coordinates;
	}
};
