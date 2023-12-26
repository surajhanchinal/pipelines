#pragma once

class LineQueue {

private:
	typedef struct {
		char *line;
		void *next;
	} Line;

	Line *first = NULL, *last = NULL;
	int _size = 0;
	
public:
	void add(char* line) {
		Line *l = new Line;
		l->line = line;
		l->next = NULL;
		if(last != NULL) last->next = l;
		last = l;
		if(first == NULL) first = last;
		_size++;
	}
	
	char* get() {
		Line *l = first;
		first = (Line*) l->next;
		if(first == NULL) last = NULL;
		char *line = l->line;
		delete l;
		_size--;
		return line;
	}

	boolean isEmpty() {
		return first == NULL;
	}

	int size() {return _size;}
	int firstLineLength() {return strlen(first->line);}
	
};