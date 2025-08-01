#include "conway.hpp"

void testBlock()
{
	Node* root = new Node(1);
	// 方块多次tick结果不变
	root->insert(0x1000, 0x1000);
	root->insert(0x1000, 0x1001);
	root->insert(0x1001, 0x1000);
	root->insert(0x1001, 0x1001);

	root->tick();

	assert(root->contains(0x1000, 0x1000));
	assert(root->contains(0x1000, 0x1001));
	assert(root->contains(0x1001, 0x1000));
	assert(root->contains(0x1001, 0x1001));

	delete root;
	std::cout << "Block test passed\n";
}

void testSimpleBlinker()
{
	Node* root = new Node(1);
	// 三细胞
	root->insert(1, 1);
	root->insert(2, 1);
	root->insert(3, 1);

	// 变垂直
	root->tick();
	root->PrintAll();
	assert(!root->contains(1, 1));
	assert(!root->contains(3, 1));
	assert(root->contains(2, 2));
	assert(root->contains(2, 1));
	assert(root->contains(2, 0));

	root->tick();
	root->PrintAll();
	assert(!root->contains(2, 2));
	assert(!root->contains(2, 2));
	assert(root->contains(1, 1));
	assert(root->contains(2, 1));
	assert(root->contains(3, 1));

	delete root;
	std::cout << "Simple Blinker test passed\n";
}

void testBoundaryBlinker()
{
	Node* root = new Node(1);
	root->insert(0x1001, 0x1000);
	root->insert(0x1002, 0x1000);
	root->insert(0x1003, 0x1000);

	root->PrintAll();
	root->pre_tick();
	root->PrintAll();
	root->tick();
	root->PrintAll();
	root->post_tick();
	root->PrintAll();
	assert(!root->contains(0x1001, 0x1000));
	assert(!root->contains(0x1003, 0x1000));
	assert(root->contains(0x1002, 0x1000));
	assert(root->contains(0x1002, 0x1001));
	assert(root->contains(0x1002, 0x0FFF));

	root->pre_tick();
	root->tick();
	root->post_tick();
	root->PrintAll();
	assert(root->contains(0x1001, 0x1000));
	assert(root->contains(0x1002, 0x1000));
	assert(root->contains(0x1003, 0x1000));
	assert(!root->contains(0x1002, 0x1001));
	assert(!root->contains(0x1002, 0x0FFF));

	delete root;
	std::cout << "Blinker test passed\n";
}

void testGlider() {
	Node* root = new Node(1);
	// 滑翔机
	root->insert(0x1001, 0x1000);
	root->insert(0x1002, 0x1001);
	root->insert(0x1000, 0x1002);
	root->insert(0x1001, 0x1002);
	root->insert(0x1002, 0x1002);

	root->pre_tick();
	root->tick();
	root->post_tick();

	assert(!root->contains(0x1001, 0x1000));
	assert(!root->contains(0x1000, 0x1002));
	assert(root->contains(0x1002, 0x1001));
	assert(root->contains(0x1001, 0x1002));
	assert(root->contains(0x1002, 0x1002));
	assert(root->contains(0x1000, 0x1001));
	assert(root->contains(0x1001, 0x1003));

	delete root;
	std::cout << "Glider test passed\n";
}

int main()
{
	testBlock();
	testSimpleBlinker();
	testBoundaryBlinker();
	testGlider();
	return 0;
}