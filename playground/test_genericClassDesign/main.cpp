#include <iostream>
#include <string>
#include <vector>

using namespace std;





class MyPropertyBase
{
public:
	MyPropertyBase() : x_("Hello ") {}
	const string& getX() { return x_; }

private:
	string x_;
};



class MyPropertyA : public MyPropertyBase
{
public:
	MyPropertyA() : y_("Europe!"){}
	const string& getY() { return y_; }

private:
	string y_;
};



class MyPropertyB : public MyPropertyBase
{
public:
	MyPropertyB() : z_("World!"){}
	const string& getZ() { return z_; }

private:
	string z_;
};


class MyPropertyC : public MyPropertyBase
{
public:
	MyPropertyC() : q_("Milky Way!"){}
	const string& getQ() { return q_; }

private:
	string q_;
};



class MyClassBase
{
public:
	MyClassBase() {}
	virtual void go() = 0;
};



template<typename TypeA, typename TypeB>
class MyClass : public MyClassBase
{
public:

	MyClass(TypeA* a, TypeB* b) : {}
	void init(TypeA* first, TypeB* second);
	void go();

private:
	TypeA* first_;
	TypeB* second_;
};


template<typename TypeA, typename TypeB>
void MyClass<TypeA, TypeB>::init(TypeA* first, TypeB* second)
{
	first_ = first;
	second_ = second;
}





template<> void MyClass<MyPropertyA, MyPropertyB>::go()
{ cout << first_->getX() << first_->getY() << ", " << second_->getX() << second_->getZ() << "\n"; }

template<> void MyClass<MyPropertyB, MyPropertyA>::go()
{ cout << first_->getX() << first_->getZ() << ", " << second_->getX() << second_->getY() << "\n"; }

template<> void MyClass<MyPropertyA, MyPropertyC>::go()
{ cout << first_->getX() << first_->getY() << ", " << second_->getX() << second_->getQ() << "\n"; }

template<> void MyClass<MyPropertyC, MyPropertyA>::go()
{ cout << first_->getX() << first_->getQ() << ", " << second_->getX() << second_->getX() << "\n"; }

template<> void MyClass<MyPropertyB, MyPropertyC>::go()
{ cout << first_->getX() << first_->getZ() << ", " << second_->getX() << second_->getQ() << "\n"; }

template<> void MyClass<MyPropertyC, MyPropertyB>::go()
{ cout << first_->getX() << first_->getQ() << ", " << second_->getX() << second_->getZ() << "\n"; }








MyClassBase* buildMyClass(const string& arg1, const string& arg2)
{
	if (arg1.compare("a") && arg2.compare("b"))
	{
		MyClass<MyPropertyA, MyPropertyB>* my_class = 
			new MyClass<MyPropertyA, MyPropertyB>();
		my_class->init(new MyPropertyA(), new MyPropertyB());
		return my_class;
	}

	else if (arg1.compare("b") && arg2.compare("a"))
	{
		MyClass<MyPropertyB, MyPropertyA>* my_class = 
			new MyClass<MyPropertyB, MyPropertyA>();
		my_class->init(&b, &a);
		return my_class;
	}

	else if (arg1.compare("a") && arg2.compare("c"))
	{
		MyClass<MyPropertyA, MyPropertyC>* my_class = 
			new MyClass<MyPropertyA, MyPropertyC>();
		my_class->init(&a, &c);
		return my_class;
	}

	else if (arg1.compare("c") && arg2.compare("a"))
	{
		MyClass<MyPropertyC, MyPropertyA>* my_class = 
			new MyClass<MyPropertyC, MyPropertyA>();
		my_class->init(&c, &a);
		return my_class;
	}
	
	else if (arg1.compare("b") && arg2.compare("c"))
	{
		MyClass<MyPropertyB, MyPropertyC>* my_class = 
			new MyClass<MyPropertyB, MyPropertyC>();
		my_class->init(&b, &c);
		return my_class;
	}
	
	else if (arg1.compare("c") && arg2.compare("b"))
	{
		MyClass<MyPropertyC, MyPropertyB>* my_class = 
			new MyClass<MyPropertyC, MyPropertyB>();
		my_class->init(&c, &b);
		return my_class;
	}

	return nullptr;
}



int main(int argc, char* argv[])
{

	if (argc != 3) return -1;

	string arg1 = argv[1];
	string arg2 = argv[2];

	MyPropertyA a;
	MyPropertyB b;
	MyPropertyC c;

	vector<MyClassBase*> my_classes;

	MyClassBase* my_class = buildMyClass(arg1, arg2);
	if (my_class == nullptr) return -2;

	my_classes.push_back(my_class);
	
	my_classes.at(0)->go();
	
	delete my_classes.at(0);
	
	return 0;
}