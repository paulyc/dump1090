# dump1090 - ADS-B software radio decoder
#
# Copyright (C) 2019 Paul Ciarlo <paul.ciarlo@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

add_executable(dump1090 dump1090-main.c)
target_link_libraries(dump1090 1090)
add_executable(faup1090 faup1090-main.c)
target_link_libraries(faup1090 1090)
add_executable(view1090 view1090-main.c)
target_link_libraries(view1090 1090)
