#include "robot_model_repository.h"

#include "pugixml.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>
#include <utility>

namespace robot_model
{
  namespace
  {

    int item_value_as_int(pugi::xml_node root, const char *key, int fallback)
    {
      const auto xpath = std::string(".//Item[@keyname='") + key + "']";
      const auto node = root.select_node(xpath.c_str()).node();
      if (!node)
      {
        return fallback;
      }
      return node.attribute("value").as_int(fallback);
    }

    std::string item_value_as_string(pugi::xml_node root,
                                     const char *key,
                                     const char *fallback)
    {
      const auto xpath = std::string(".//Item[@keyname='") + key + "']";
      const auto node = root.select_node(xpath.c_str()).node();
      if (!node)
      {
        return fallback;
      }
      return node.attribute("value").as_string(fallback);
    }

    bool extension_equals(const std::filesystem::path &path,
                          std::string_view expected)
    {
      const auto extension = path.extension().string();
      if (extension.size() != expected.size())
      {
        return false;
      }

      return std::equal(extension.begin(),
                        extension.end(),
                        expected.begin(),
                        [](unsigned char lhs, unsigned char rhs)
                        {
                          return std::tolower(lhs) == std::tolower(rhs);
                        });
    }

    bool natural_stl_less(const std::filesystem::path &lhs,
                          const std::filesystem::path &rhs)
    {
      const auto left = lhs.stem().string();
      const auto right = rhs.stem().string();

      const auto left_is_number = !left.empty() &&
                                  std::all_of(left.begin(), left.end(),
                                              [](unsigned char c)
                                              { return std::isdigit(c); });
      const auto right_is_number = !right.empty() &&
                                   std::all_of(right.begin(), right.end(),
                                               [](unsigned char c)
                                               { return std::isdigit(c); });

      if (left_is_number && right_is_number)
      {
        return std::stoi(left) < std::stoi(right);
      }

      return lhs.filename().string() < rhs.filename().string();
    }

    std::filesystem::path first_xml_in_dir(const std::filesystem::path &dir)
    {
      const auto robot_xml = dir / "robot.xml";
      if (std::filesystem::exists(robot_xml))
      {
        return robot_xml;
      }

      for (const auto &entry : std::filesystem::directory_iterator(dir))
      {
        if (entry.is_regular_file() && extension_equals(entry.path(), ".xml"))
        {
          return entry.path();
        }
      }

      return {};
    }

    Robot_Model_Info describe_robot_model(const std::filesystem::path &dir)
    {
      Robot_Model_Info info;
      info.model_dir = dir;
      info.display_name = dir.filename().string();
      info.xml_path = first_xml_in_dir(dir);

      pugi::xml_document doc;
      pugi::xml_node root;
      if (!info.xml_path.empty() &&
          std::filesystem::exists(info.xml_path) &&
          doc.load_file(info.xml_path.wstring().c_str()))
      {
        root = doc.child("RobotTemplate");
      }

      if (root)
      {
        const auto name = item_value_as_string(root, "ManipulatorModel", info.display_name.c_str());
        if (!name.empty())
        {
          info.display_name = name;
        }
      }

      for (const auto &entry : std::filesystem::directory_iterator(dir))
      {
        if (entry.is_regular_file() && extension_equals(entry.path(), ".stl"))
        {
          info.stl_files.push_back(entry.path());
        }
      }

      std::sort(info.stl_files.begin(),
                info.stl_files.end(),
                natural_stl_less);

      if (root)
      {
        std::vector<std::filesystem::path> xml_stl_files;
        const auto model_number = item_value_as_int(root, "ModelNumber", static_cast<int>(info.stl_files.size()));
        for (int i = 1; i <= model_number; ++i)
        {
          const auto key = std::string("Model_") + std::to_string(i);
          const auto file_name = item_value_as_string(root, key.c_str(), "");
          if (file_name.empty())
          {
            continue;
          }

          const auto stl_path = dir / file_name;
          if (std::filesystem::exists(stl_path))
          {
            xml_stl_files.push_back(stl_path);
          }
        }

        if (!xml_stl_files.empty())
        {
          info.stl_files = std::move(xml_stl_files);
        }
      }

      return info;
    }

    bool contains_loadable_robot_model(const std::filesystem::path &root)
    {
      if (!std::filesystem::is_directory(root))
      {
        return false;
      }

      for (const auto &entry : std::filesystem::directory_iterator(root))
      {
        if (!entry.is_directory())
        {
          continue;
        }

        const auto model = describe_robot_model(entry.path());
        if (!model.xml_path.empty() && !model.stl_files.empty())
        {
          return true;
        }
      }
      return false;
    }

  } // namespace

  std::filesystem::path Find_Robot_Root()
  {
    const std::filesystem::path root(ROBOT_RESOURCE_ROOT);
    if (contains_loadable_robot_model(root))
    {
      return root;
    }

    return {};
  }

  std::vector<Robot_Model_Info> Scan_Models_In_Directory(const std::filesystem::path &root)
  {
    std::vector<Robot_Model_Info> models;

    if (root.empty() || !std::filesystem::exists(root))
    {
      return models;
    }

    for (const auto &entry : std::filesystem::directory_iterator(root))
    {
      if (!entry.is_directory())
      {
        continue;
      }

      auto info = describe_robot_model(entry.path());
      if (info.stl_files.empty())
      {
        continue;
      }

      models.push_back(std::move(info));
    }

    return models;
  }

} // namespace robot_model
