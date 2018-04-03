#pragma once
#include <vector>

template<typename T>
void deleteStdvectorPointer(std::vector<T> &pointer_list)
{
	for (std::vector<T>::iterator it = pointer_list.begin(); it != pointer_list.end(); ++it)
	{
		delete (*it);
	}
	pointer_list.clear();
}

template<typename T>
void deleteStdvectorPointer(std::vector<T> *pointer_list)
{
	for (std::vector<T>::iterator it = pointer_list->begin(); it != pointer_list->end(); ++it)
	{
		delete (*it);
	}
	delete pointer_list;
}