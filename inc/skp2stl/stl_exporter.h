/**
 * @file	stl_exporter.h
 * @author	Joseph Lee <joseph@jc-lab.net>
 * @date	2021/05/29
 * @copyright Copyright (C) 2021 jc-lab.\n
 *            This software may be modified and distributed under the terms
 *            of the Apache License 2.0.  See the LICENSE file for details.
 */

#ifndef SKP2STL_STL_EXPORTER_H_
#define SKP2STL_STL_EXPORTER_H_

#include <memory>
#include <fstream>

namespace skp2stl {

class StlExporter {
 public:
  static std::unique_ptr<StlExporter> create();

  virtual int loadFromEntities(SUEntitiesRef entities) = 0;
  virtual int exportToStl(std::fstream& stream, const char* project_name) const = 0;
};

} // namespace skp2stl

#endif //SKP2STL_STL_EXPORTER_H_
