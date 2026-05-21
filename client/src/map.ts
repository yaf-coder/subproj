import maplibregl, { Map, Marker, LngLatLike } from "maplibre-gl";
import type { BathyGrid, CurrentsGrid, Plan } from "./api";

const STYLE = {
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

export interface MapHandle {
  map: Map;
  setStart(ll: LngLatLike | null): void;
  setGoal(ll: LngLatLike | null): void;
  setRoute(plan: Plan | null): void;
  setSub(lng: number, lat: number, heading_deg: number): void;
  appendTrack(lng: number, lat: number): void;
  clearTrack(): void;
  setBathymetry(grid: BathyGrid | null): void;
  setBathymetryVisible(v: boolean): void;
  setCurrents(grid: CurrentsGrid | null): void;
  setCurrentsVisible(v: boolean): void;
}

export function initMap(container: HTMLElement, onClick: (lng: number, lat: number) => void): MapHandle {
  const map = new maplibregl.Map({
    container,
    style: STYLE,
    center: [-121.89, 36.61],
    zoom: 13,
    pitch: 0,
  });

  let startMarker: Marker | null = null;
  let goalMarker: Marker | null = null;
  let subMarker: Marker | null = null;
  const trackCoords: [number, number][] = [];
  const currentMarkers: Marker[] = [];
  let bathyVisible = true;
  let currentsVisible = true;

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

  // ----- Currents: SVG arrow marker -------------------------------------
  function arrowEl(u: number, v: number, magMax: number): HTMLElement {
    const el = document.createElement("div");
    const mag = Math.hypot(u, v);

    // CSS rotate() is clockwise-positive in a y-down screen coord system.
    // To make a right-pointing line represent a current vector (u east, v
    // north), we want CSS rotation = atan2(-v, u). In degrees that equals
    // (bearing - 90), where bearing = atan2(u, v) is the standard
    // compass bearing (0 = N, 90 = E).
    const rotation_deg =
      (Math.atan2(-v, u) * 180) / Math.PI;

    const length = 8 + 24 * Math.min(1, mag / Math.max(0.05, magMax));
    el.style.cssText = `
      position: relative;
      width: ${length}px; height: 2px;
      background: linear-gradient(to right, rgba(255,255,255,0.35) 0%, #ffe070 100%);
      border-radius: 1px;
      transform: rotate(${rotation_deg}deg);
      transform-origin: 0% 50%;
      pointer-events: none;
    `;
    // Arrowhead at the tip.
    const head = document.createElement("span");
    head.style.cssText = `
      position: absolute;
      left: ${length - 6}px; top: -3px;
      width: 0; height: 0;
      border-left: 7px solid #ffe070;
      border-top: 4px solid transparent;
      border-bottom: 4px solid transparent;
    `;
    el.appendChild(head);
    // Small dot at the tail to mark the actual sample location.
    const tail = document.createElement("span");
    tail.style.cssText = `
      position: absolute;
      left: -2px; top: -2px;
      width: 5px; height: 5px;
      border-radius: 50%;
      background: rgba(255, 224, 112, 0.55);
    `;
    el.appendChild(tail);
    return el;
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

    setCurrents(grid) {
      // Remove old markers and recreate. With ~12x8 = 96 markers this is
      // cheap; if it ever isn't we can diff.
      for (const m of currentMarkers) m.remove();
      currentMarkers.length = 0;
      if (!grid || !currentsVisible) return;
      // Find max magnitude for arrow scaling.
      let magMax = 0;
      for (let k = 0; k < grid.u.length; k++) {
        const m = Math.hypot(grid.u[k], grid.v[k]);
        if (m > magMax) magMax = m;
      }
      magMax = Math.max(0.05, magMax);
      for (let j = 0; j < grid.height; j++) {
        const lat = grid.lat_min + ((grid.lat_max - grid.lat_min) * (j + 0.5)) / grid.height;
        for (let i = 0; i < grid.width; i++) {
          const lon = grid.lon_min + ((grid.lon_max - grid.lon_min) * (i + 0.5)) / grid.width;
          const idx = j * grid.width + i;
          const u = grid.u[idx];
          const v = grid.v[idx];
          if (Math.hypot(u, v) < 0.005) continue;
          const el = arrowEl(u, v, magMax);
          // Anchor "left" puts the lat/lon at the LEFT-CENTER of the marker
          // element, which is the arrow's tail (the geometrically correct
          // place for a vector-field sample).
          const marker = new maplibregl.Marker({ element: el, anchor: "left" })
            .setLngLat([lon, lat])
            .addTo(map);
          currentMarkers.push(marker);
        }
      }
    },

    setCurrentsVisible(v) {
      currentsVisible = v;
      for (const m of currentMarkers) {
        const el = m.getElement();
        el.style.visibility = v ? "visible" : "hidden";
      }
    },
  };
}
