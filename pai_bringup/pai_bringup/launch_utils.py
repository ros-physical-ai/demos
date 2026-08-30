# Copyright 2026 ROS Physical AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch helpers shared by the pai_* bringup launch files.

`ReplaceString` is a minimal stand-in for `nav2_common.launch.ReplaceString`,
which is not published for robostack-lyrical. It only covers what the demos
use: textual placeholder substitution in a parameter file.
"""

import tempfile

from launch.substitution import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


class ReplaceString(Substitution):
    """Substitution that copies a file, replacing literal text along the way.

    Both the source file and the replacement values may themselves be
    substitutions; they are resolved when the launch context is evaluated.
    Returns the path of the rewritten temporary file.
    """

    def __init__(self, source_file, replacements):
        super().__init__()
        self.__source_file = source_file
        self.__replacements = replacements

    @property
    def source_file(self):
        """Get the source file, as a list of substitutions."""
        return self.__source_file

    @property
    def replacements(self):
        """Get the replacement mapping."""
        return self.__replacements

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return f"ReplaceString({self.__source_file}, {self.__replacements})"

    def perform(self, context) -> str:
        """Rewrite the source file into a temporary file and return its path."""
        source_path = perform_substitutions(context, normalize_to_list_of_substitutions(self.__source_file))
        with open(source_path) as source:
            contents = source.read()

        for key, value in self.__replacements.items():
            resolved_key = perform_substitutions(context, normalize_to_list_of_substitutions(key))
            resolved_value = perform_substitutions(context, normalize_to_list_of_substitutions(value))
            contents = contents.replace(resolved_key, resolved_value)

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as rewritten:
            rewritten.write(contents)
            return rewritten.name
