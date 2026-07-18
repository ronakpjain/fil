# Documentation Theme

The static API site uses the **sidebar-only layout** from [Doxygen Awesome CSS](https://github.com/jothepro/doxygen-awesome-css), version **2.4.2**. It is distributed under the MIT license; the vendored license is at `docs/theme/LICENSE.doxygen-awesome-css`.

`docs/theme/fil-theme.css` is a small override layer inspired by [ronakpjain.com](https://ronakpjain.com):

- background: `#1e1e2e`;
- primary/accent text: `#fab387`;
- monospace family: Hack;
- low-radius terminal-like panels and borders.

The Hack Regular and Bold webfonts are copied into the generated static site, so opening `docs/html/index.html` works without a network connection. The theme itself remains upstream Doxygen Awesome CSS; project CSS only changes variables and small visual details.

To regenerate after changing documentation or styles:

```bash
cmake --build build --target docs
open docs/html/index.html
```
