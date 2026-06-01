// @ts-check
import { defineConfig } from "astro/config";
import starlight from "@astrojs/starlight";

// https://astro.build/config
export default defineConfig({
  site: "https://biomelabs.github.io/openstride",
  base: "/openstride",
  integrations: [
    starlight({
      title: "OpenStride",
      social: [
        {
          icon: "github",
          label: "GitHub",
          href: "https://github.com/biomelabs/openstride",
        },
      ],
      sidebar: [
        {
          label: "Setup",
          items: [
            { label: "Overview", slug: "setup" },
            { label: "Environment", slug: "setup/environment" },
            { label: "Workspace", slug: "setup/workspace" },
            { label: "Build & Flash", slug: "setup/build-and-flash" },
            { label: "Simulation", slug: "setup/simulation" },
            { label: "ANT+ SDK", slug: "setup/ant-plus" },
          ],
        },
        { label: "Hardware", slug: "hardware" },
        // {
        //   label: "Reference",
        //   items: [{ autogenerate: { directory: "reference" } }],
        // },
      ],
    }),
  ],
});
