

#include <iostream>
#include <algorithm>

#include <utils/tokenizer.hpp>
#include <utils/static_vector.hpp>



int main_1 (void)
{
  std::cout << "\nmain 1" << std::endl;

  const char* test = "123 123";

  auto tokens = utils::tokenize (test, utils::char_separator<char> (' '));

  for (auto&& t : tokens)
    std::cout << "token: " << t << std::endl;


//  std::cout << "" << utils::is_iterator<const char*>::value << std::endl;;
}

int main_2 (void)
{
  std::cout << "\nmain 2" << std::endl;

  std::string test = "a";

  utils::tokenizer<std::string::const_iterator,
		   utils::char_separator<char>> tokens (test, ' ',
		   utils::drop_empty_tokens);

  std::cout << "size: " << tokens.size () << std::endl;

  for (auto&& t : tokens)
  {
    std::cout << "token: " << t << std::endl;
  }

  utils::static_vector<std::string_view, 4> tokens2;

  std::copy_if (tokens.begin (), tokens.end (), std::back_inserter (tokens2),
		[&] (const auto&) { return tokens2.size () < tokens2.capacity (); });

  std::cout << " tokens 2  size: " << tokens2.size () << std::endl;

  for (auto&& t : tokens2)
  {
    std::cout << "token: " << t << std::endl;

  }


/*
  auto i = tokens.begin ();

  if (i == tokens.end ())
    std::cout << "empty" << std::endl;
  else
    std::cout << "token: " << *i << std::endl;
*/
  return 0;
}

int main_3 (void)
{
  std::cout << "\nmain 3" << std::endl;

  const char* test_raw = "  123 23345 9 a k  memt ";

  std::string_view test (test_raw);

  auto tokens = utils::tokenize (test, utils::char_separator<char> (' '));

  for (auto&& t : tokens)
    std::cout << "token: " << t << std::endl;
}

int main_4 (void)
{
  std::cout << "\nmain 4" << std::endl;

  const char* test_raw = "  123 23345 9 a k  memt ";

  std::string_view test (test_raw);

  auto tokens = utils::tokenize (test, utils::char_separator<char> (' ',
			utils::keep_empty_tokens));

  for (auto&& t : tokens)
    std::cout << "token: " << t << std::endl;
}


int main_5 (void)
{
  std::cout << "\nmain 5" << std::endl;

  const char* test_raw = " 123 123 ";
//                        ^
//                           ^

//                           ^
//                           ^

//                            ^
//                            ^

//                            ^
//                               ^

//                               ^
//                               ^



  std::string_view test (test_raw);

  auto tokens = utils::tokenize (test,
	utils::char_separator<char> (' ', utils::keep_empty_tokens));

  for (auto&& t : tokens)
    std::cout << "token: " << t << std::endl;
}

int main_6 (void)
{
  std::cout << "\nmain 6" << std::endl;

  const char* test_raw = " ";

  auto tokens = utils::tokenize (test_raw,
	utils::char_separator<char> (' ', utils::keep_empty_tokens));

  for (auto&& t : tokens)
    std::cout << "token: " << t << std::endl;
}

int main_7 (void)
{
  std::cout << "\nmain 7" << std::endl;

  const char* test_raw = " ";
//                        ^
//                        ^

//                         ^
//                         ^

  auto tokens = utils::tokenize (test_raw,
	utils::char_separator<char> (' ', utils::drop_empty_tokens));

  for (auto&& t : tokens)
    std::cout << "token: " << t << std::endl;
}

int main_8 (void)
{
  std::cout << "\nmain 8" << std::endl;

  const char* test_raw = "  ";

  auto tokens = utils::tokenize (test_raw,
	utils::char_separator<char> (' ', utils::keep_empty_tokens));

  for (auto&& t : tokens)
    std::cout << "token: " << t << std::endl;
}


int main (void)
{
  main_1 ();
  main_2 ();
  main_3 ();
  main_4 ();
  main_5 ();
  main_6 ();
  main_7 ();
  main_8 ();
  return 0;
}
