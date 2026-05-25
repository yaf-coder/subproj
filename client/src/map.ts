import maplibregl, { Map, Marker, LngLatLike } from "maplibre-gl";
import type { BathyGrid, CurrentsGrid, Plan } from "./api";

const STYLE_2D = {
  version: 8 as const,
  sources: {
    osm: {
      type: "raster" as const,
      tiles: ["https://tile.openstreetmap.org/{z}/{x}/{y}.png"],
      tileSize: 256,
      attribution: "&copy; OpenStreetMap contributors",
    },
  },
  layers: [{ id: "osm", type: "raster" as const, source: "osm" }],
};

// 3D-terrain style: OSM base tiles + the AWS Open Data "Terrain Tiles" DEM
// (a public, no-auth, terrarium-encoded global elevation source that
// includes bathymetry). MapLibre uses the DEM both to displace map tiles
// vertically when `setTerrain` is enabled and to render hillshading.
const STYLE_3D = {
  version: 8 as const,
  sources: {
    osm: {
      type: "raster" as const,
      tiles: ["https://tile.openstreetmap.org/{z}/{x}/{y}.png"],
      tileSize: 256,
      attribution: "&copy; OpenStreetMap contributors",
    },
    terrain: {
      type: "raster-dem" as const,
      tiles: [
        "https://elevation-tiles-prod.s3.amazonaws.com/terrarium/{z}/{x}/{y}.png",
      ],
      tileSize: 256,
      encoding: "terrarium" as const,
      maxzoom: 12,
      attribution:
        "Terrain tiles &copy; Mapzen / Amazon Public Datasets",
    },
  },
  layers: [
    { id: "osm", type: "raster" as const, source: "osm" },
    // Soft hillshading on top of the base raster so the terrain reads as 3D
    // even when the tilted view's perspective is shallow.
    {
      id: "hillshade",
      type: "hillshade" as const,
      source: "terrain",
      paint: {
        "hillshade-shadow-color": "#0a1620",
        "hillshade-highlight-color": "#e4f1ff",
        "hillshade-accent-color": "#3a78a0",
        "hillshade-exaggeration": 0.7,
      },
    },
  ],
};

// Draw an "arrow pointing up" (north) into a small canvas and return its
// ImageData. The symbol layer rotates this base image per-feature using
// `icon-rotate` (clockwise compass-bearing degrees), so the source image
// needs to point north at rotate=0.
function makeArrowImageData(): ImageData {
  const W = 20;
  const H = 50;
  const canvas = document.createElement("canvas");
  canvas.width = W;
  canvas.height = H;
  const ctx = canvas.getContext("2d")!;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#ffe070";
  // Body: vertical line from y=H-4 (just above the bottom) up to y=14
  // (just below the arrowhead).
  ctx.fillRect(W / 2 - 1.5, 14, 3, H - 18);
  // Arrowhead at the top.
  ctx.beginPath();
  ctx.moveTo(W / 2, 2);
  ctx.lineTo(W / 2 - 7, 16);
  ctx.lineTo(W / 2 + 7, 16);
  ctx.closePath();
  ctx.fill();
  // Tail dot at the bottom — marks the sample point.
  ctx.beginPath();
  ctx.arc(W / 2, H - 3, 2.5, 0, Math.PI * 2);
  ctx.fill();
  return ctx.getImageData(0, 0, W, H);
}

export interface InitMapOpts {
  // 3D-terrain view: tilts the camera, enables terrain raster-dem source.
  tilted?: boolean;
  center?: [number, number];
  zoom?: number;
  pitch?: number;
  bearing?: number;
  // For tilted maps, scales the vertical exaggeration of the terrain.
  // 4x makes underwater features clearly readable; 1x is geometrically true
  // but the seafloor profile looks visually flat at typical zooms.
  terrainExaggeration?: number;
}

export interface MapHandle {
  map: Map;
  setStart(ll: LngLatLike | null): void;
  setGoal(ll: LngLatLike | null): void;
  setRoute(plan: Plan | null): void;
  setSub(lng: number, lat: number, heading_deg: number): void;
  appendTrack(lng: number, lat: number): void;
  setTrack(coords: [number, number][]): void;
  clearTrack(): void;
  setBathymetry(grid: BathyGrid | null): void;
  setBathymetryVisible(v: boolean): void;
  setCurrents(grid: CurrentsGrid | null): void;
  setCurrentsVisible(v: boolean): void;
}

export function initMap(
  container: HTMLElement,
  onClick: (lng: number, lat: number) => void,
  opts: InitMapOpts = {},
): MapHandle {
  const tilted = !!opts.tilted;
  const map = new maplibregl.Map({
    container,
    style: tilted ? STYLE_3D : STYLE_2D,
    center: opts.center ?? [-121.89, 36.61],
    zoom: opts.zoom ?? (tilted ? 10 : 13),
    pitch: opts.pitch ?? (tilted ? 62 : 0),
    bearing: opts.bearing ?? (tilted ? 20 : 0),
    maxPitch: tilted ? 75 : 60,
  });

  if (tilted) {
    map.on("load", () => {
      map.setTerrain({
        source: "terrain",
        exaggeration: opts.terrainExaggeration ?? 4,
      });
    });
  }

  let startMarker: Marker | null = null;
  let goalMarker: Marker | null = null;
  let subMarker: Marker | null = null;
  const trackCoords: [number, number][] = [];
  let bathyVisible = true;

  map.on("load", () => {
    // Bathymetry image source — added/updated lazily by setBathymetry().
    // We can't create the source with empty coordinates so we wait until the
    // first grid arrives.

    map.addSource("route", { type: "geojson", data: emptyLine() });
    map.addLayer({
      id: "route-line",
      type: "line",
      source: "route",
      paint: {
        "line-color": "#4cc4ff",
        "line-width": 3,
        "line-dasharray": [2, 2],
      },
    });
    map.addSource("route-waypoints", { type: "geojson", data: emptyFC() });
    map.addLayer({
      id: "route-waypoints-dots",
      type: "circle",
      source: "route-waypoints",
      paint: {
        "circle-radius": 3,
        "circle-color": "#4cc4ff",
        "circle-opacity": 0.45,
        "circle-stroke-color": "#001827",
        "circle-stroke-width": 0.5,
        "circle-stroke-opacity": 0.6,
      },
    });
    map.addSource("track", { type: "geojson", data: emptyLine() });
    map.addLayer({
      id: "track-line",
      type: "line",
      source: "track",
      paint: { "line-color": "#ffa000", "line-width": 2 },
    });

    // Register a per-pixel "arrow" image and add the currents symbol layer.
    // The arrow is drawn vertically (pointing UP / north) so that
    // icon-rotate=bearing degrees rotates clockwise from the natural
    // north-pointing orientation — which matches MapLibre's symbol
    // rotation convention out of the box.
    if (!map.hasImage("current-arrow")) {
      map.addImage("current-arrow", makeArrowImageData());
    }
    map.addSource("currents", { type: "geojson", data: emptyFC() });
    map.addLayer({
      id: "currents-layer",
      type: "symbol",
      source: "currents",
      layout: {
        "icon-image": "current-arrow",
        "icon-rotate": ["get", "bearing"],
        "icon-size": ["get", "mag_scale"],
        // icon-anchor: bottom puts the lat/lon at the tail of the arrow
        // (the natural vector-field convention: tail at sample point,
        // head pointing in flow direction).
        "icon-anchor": "bottom",
        "icon-rotation-alignment": "map",
        "icon-pitch-alignment": "map",
        // Allow every sample to render regardless of icon overlap;
        // currents are a regular grid so overlap is expected.
        "icon-allow-overlap": true,
        "icon-ignore-placement": true,
      },
      paint: {
        "icon-opacity": 0.7,
      },
    });
  });

  map.on("click", (e) => onClick(e.lngLat.lng, e.lngLat.lat));

  function emptyLine(): GeoJSON.FeatureCollection {
    return { type: "FeatureCollection", features: [] };
  }
  function emptyFC(): GeoJSON.FeatureCollection {
    return { type: "FeatureCollection", features: [] };
  }
  function lineFC(coords: [number, number][]): GeoJSON.FeatureCollection {
    return {
      type: "FeatureCollection",
      features: [{
        type: "Feature",
        properties: {},
        geometry: { type: "LineString", coordinates: coords },
      }],
    };
  }
  function pointsFC(coords: [number, number][]): GeoJSON.FeatureCollection {
    return {
      type: "FeatureCollection",
      features: coords.map((c) => ({
        type: "Feature",
        properties: {},
        geometry: { type: "Point", coordinates: c },
      })),
    };
  }

  function colorPin(color: string): HTMLElement {
    const el = document.createElement("div");
    el.style.cssText = `
      width: 16px; height: 16px; border-radius: 50%;
      background: ${color}; border: 2px solid #001827;
      box-shadow: 0 0 0 2px ${color}88;
    `;
    return el;
  }

  function subPin(): HTMLElement {
    const el = document.createElement("div");
    el.style.cssText = `
      width: 0; height: 0;
      border-left: 8px solid transparent;
      border-right: 8px solid transparent;
      border-bottom: 18px solid #4cc4ff;
      filter: drop-shadow(0 0 4px #4cc4ff88);
      transform-origin: 50% 70%;
    `;
    return el;
  }

  // ----- Bathymetry: canvas rasterization to a data URL ------------------
  function bathyCanvas(grid: BathyGrid): HTMLCanvasElement {
    const canvas = document.createElement("canvas");
    canvas.width = grid.width;
    canvas.height = grid.height;
    const ctx = canvas.getContext("2d")!;
    const img = ctx.createImageData(grid.width, grid.height);
    const maxd = Math.max(50, grid.max_depth_m);
    // Stretch contrast: take sqrt so shallow detail isn't crushed.
    for (let j = 0; j < grid.height; j++) {
      for (let i = 0; i < grid.width; i++) {
        const depth = grid.depths[j * grid.width + i];
        // Image row 0 = top (lat_max), but server row 0 = bottom (lat_min).
        const outRow = grid.height - 1 - j;
        const px = (outRow * grid.width + i) * 4;
        if (depth <= 0) {
          // Land — leave fully transparent so OSM tile shows through.
          img.data[px + 0] = 0;
          img.data[px + 1] = 0;
          img.data[px + 2] = 0;
          img.data[px + 3] = 0;
        } else {
          const t = Math.min(1, Math.sqrt(depth / maxd));
          // Cyan (shallow) -> deep blue (mid) -> near-black (abyss).
          const r = Math.round(120 * (1 - t) +  10 * t);
          const g = Math.round(190 * (1 - t) +  20 * t);
          const b = Math.round(225 * (1 - t) +  60 * t);
          img.data[px + 0] = r;
          img.data[px + 1] = g;
          img.data[px + 2] = b;
          // Stronger alpha in deep areas so the colormap reads.
          img.data[px + 3] = 130;
        }
      }
    }
    ctx.putImageData(img, 0, 0);
    return canvas;
  }

  return {
    map,
    setStart(ll) {
      startMarker?.remove();
      startMarker = ll ? new maplibregl.Marker({ element: colorPin("#4cc4ff") }).setLngLat(ll).addTo(map) : null;
    },
    setGoal(ll) {
      goalMarker?.remove();
      goalMarker = ll ? new maplibregl.Marker({ element: colorPin("#ffa000") }).setLngLat(ll).addTo(map) : null;
    },
    setRoute(plan) {
      const lineSrc = map.getSource("route") as maplibregl.GeoJSONSource | undefined;
      const wpSrc = map.getSource("route-waypoints") as maplibregl.GeoJSONSource | undefined;
      if (!lineSrc || !wpSrc) return;
      if (!plan) {
        lineSrc.setData(emptyLine());
        wpSrc.setData(emptyFC());
        return;
      }
      const coords: [number, number][] = plan.waypoints.map((w) => [w.lon, w.lat]);
      lineSrc.setData(lineFC(coords));
      // Show waypoint dots but skip the very first (sits under the start pin)
      // and the very last (sits under the goal pin).
      const dotCoords = coords.length > 2 ? coords.slice(1, -1) : [];
      wpSrc.setData(pointsFC(dotCoords));
    },
    setSub(lng, lat, heading_deg) {
      if (!subMarker) {
        subMarker = new maplibregl.Marker({ element: subPin(), rotationAlignment: "map" }).setLngLat([lng, lat]).addTo(map);
      } else {
        subMarker.setLngLat([lng, lat]);
      }
      // Heading in map: 0 deg = north, increasing clockwise. Our heading_deg is from east, CCW.
      // Convert: map_bearing = 90 - heading_deg.
      const map_bearing = 90 - heading_deg;
      subMarker.setRotation(map_bearing);
    },
    appendTrack(lng, lat) {
      trackCoords.push([lng, lat]);
      const src = map.getSource("track") as maplibregl.GeoJSONSource | undefined;
      if (src) src.setData(lineFC(trackCoords));
    },
    setTrack(coords) {
      trackCoords.length = 0;
      trackCoords.push(...coords);
      const src = map.getSource("track") as maplibregl.GeoJSONSource | undefined;
      if (src) src.setData(lineFC(trackCoords));
    },

    clearTrack() {
      trackCoords.length = 0;
      const src = map.getSource("track") as maplibregl.GeoJSONSource | undefined;
      if (src) src.setData(emptyLine());
    },

    setBathymetry(grid) {
      // Add or replace the bathymetry image source/layer.
      if (!grid) {
        if (map.getLayer("bathymetry-layer")) map.removeLayer("bathymetry-layer");
        if (map.getSource("bathymetry")) map.removeSource("bathymetry");
        return;
      }
      const canvas = bathyCanvas(grid);
      const url = canvas.toDataURL("image/png");
      const coords: [
        [number, number],
        [number, number],
        [number, number],
        [number, number],
      ] = [
        [grid.lon_min, grid.lat_max], // top-left
        [grid.lon_max, grid.lat_max], // top-right
        [grid.lon_max, grid.lat_min], // bottom-right
        [grid.lon_min, grid.lat_min], // bottom-left
      ];
      const existing = map.getSource("bathymetry") as
        | (maplibregl.ImageSource & { updateImage: (opts: { url: string; coordinates: typeof coords }) => void })
        | undefined;
      if (existing) {
        existing.updateImage({ url, coordinates: coords });
      } else {
        map.addSource("bathymetry", { type: "image", url, coordinates: coords });
        // Place above OSM but below route/track.
        const beforeId = map.getLayer("route-line") ? "route-line" : undefined;
        map.addLayer({
          id: "bathymetry-layer",
          type: "raster",
          source: "bathymetry",
          paint: { "raster-opacity": bathyVisible ? 0.7 : 0 },
        }, beforeId);
      }
    },

    setBathymetryVisible(v) {
      bathyVisible = v;
      if (map.getLayer("bathymetry-layer")) {
        map.setPaintProperty("bathymetry-layer", "raster-opacity", v ? 0.7 : 0);
      }
    },

    // ----- Currents (symbol layer, not HTML markers) ---------------------
    //
    // Currents are rendered as a MapLibre symbol layer backed by a GeoJSON
    // point source. Each sample is a Point feature with `bearing` and
    // `mag_scale` properties; the symbol layer rotates and scales an
    // "arrow" icon image per-feature. The icon is registered once on map
    // load. Because the symbol layer is part of the map's WebGL render
    // pipeline (not separately-positioned HTML overlays), MapLibre's own
    // projection handles the placement and there are no resize, layout, or
    // DOM-anchoring failure modes.
    setCurrents(grid) {
      const src = map.getSource("currents") as
        | maplibregl.GeoJSONSource
        | undefined;
      if (!src) return;
      if (!grid) {
        src.setData(emptyFC());
        return;
      }
      let magMax = 0;
      for (let k = 0; k < grid.u.length; k++) {
        const m = Math.hypot(grid.u[k], grid.v[k]);
        if (m > magMax) magMax = m;
      }
      magMax = Math.max(0.05, magMax);
      const features: GeoJSON.Feature[] = [];
      for (let j = 0; j < grid.height; j++) {
        const lat =
          grid.lat_min +
          ((grid.lat_max - grid.lat_min) * (j + 0.5)) / grid.height;
        for (let i = 0; i < grid.width; i++) {
          const lon =
            grid.lon_min +
            ((grid.lon_max - grid.lon_min) * (i + 0.5)) / grid.width;
          const idx = j * grid.width + i;
          const u = grid.u[idx];
          const v = grid.v[idx];
          const mag = Math.hypot(u, v);
          if (mag < 0.005) continue;
          // Compass bearing of the (u east, v north) vector. atan2(u, v)
          // gives 0 for due-north, 90 for due-east — same convention as
          // MapLibre's icon-rotate property.
          const bearing =
            ((Math.atan2(u, v) * 180) / Math.PI + 360) % 360;
          const mag_scale = 0.35 + 0.65 * Math.min(1, mag / magMax);
          features.push({
            type: "Feature",
            properties: { bearing, mag_scale },
            geometry: { type: "Point", coordinates: [lon, lat] },
          });
        }
      }
      src.setData({ type: "FeatureCollection", features });
    },

    setCurrentsVisible(v) {
      if (map.getLayer("currents-layer")) {
        map.setLayoutProperty(
          "currents-layer",
          "visibility",
          v ? "visible" : "none",
        );
      }
    },
  };
}

// Combine multiple MapHandles into one. Shared visual operations (start
// pin, goal pin, route, sub icon, track polyline) broadcast to every
// handle so all maps stay in sync. Bathymetry and current-arrow overlays
// only apply to the first handle (the 2D top-down map) — the 3D map
// already shows bathymetry through real terrain, so a flat overlay on
// top would be redundant.
export function combineHandles(handles: MapHandle[]): MapHandle {
  if (handles.length === 0) throw new Error("combineHandles needs at least one handle");
  const primary = handles[0];
  return {
    map: primary.map,
    setStart: (ll) => handles.forEach((h) => h.setStart(ll)),
    setGoal: (ll) => handles.forEach((h) => h.setGoal(ll)),
    setRoute: (plan) => handles.forEach((h) => h.setRoute(plan)),
    setSub: (lng, lat, hdg) =>
      handles.forEach((h) => h.setSub(lng, lat, hdg)),
    appendTrack: (lng, lat) =>
      handles.forEach((h) => h.appendTrack(lng, lat)),
    setTrack: (coords) => handles.forEach((h) => h.setTrack(coords)),
    clearTrack: () => handles.forEach((h) => h.clearTrack()),
    setBathymetry: (g) => primary.setBathymetry(g),
    setBathymetryVisible: (v) => primary.setBathymetryVisible(v),
    setCurrents: (g) => primary.setCurrents(g),
    setCurrentsVisible: (v) => primary.setCurrentsVisible(v),
  };
}
