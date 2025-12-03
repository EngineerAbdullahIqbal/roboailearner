import React from 'react';
import clsx from 'clsx';
import styles from './CourseOverview.module.css';

// Import Swiper React components
import { Swiper, SwiperSlide } from 'swiper/react';

// Import Swiper styles
import 'swiper/css';
import 'swiper/css/navigation';
import 'swiper/css/pagination';

// Import required modules
import { Navigation, Pagination } from 'swiper/modules';

interface ModuleProps {
  title: string;
  description: string;
  longDescription: string; // Added long description
  icon: string; // Path to SVG icon
}

function ModuleCard({ title, description, longDescription, icon }: ModuleProps) {
  return (
    <div className={styles.moduleCard}>
      <img src={icon} alt={`${title} Icon`} className={styles.moduleIcon} />
      <h3 className={styles.moduleTitle}>{title}</h3>
      <p className={styles.moduleDescription}>{description}</p>
      {longDescription && <p className={styles.moduleLongDescription}>{longDescription}</p>}
    </div>
  );
}

interface CourseOverviewProps {
  modules: ModuleProps[];
}

function CourseOverview({ modules }: CourseOverviewProps) {
  return (
    <section className={styles.courseOverviewSection}>
      <div className="container">
        <h2 className="text--center">Course Overview</h2>
        {/* Replace the grid with Swiper */}
        <Swiper
          modules={[Navigation, Pagination]}
          spaceBetween={50} // Space between slides
          slidesPerView={1} // Show one slide at a time
          navigation // Enable navigation arrows
          pagination={{ clickable: true }} // Enable pagination dots
          className={styles.mySwiper} // Apply custom styling if needed
        >
          {modules.map((module, idx) => (
            <SwiperSlide key={idx} className={styles.moduleSwiperSlide}>
              <ModuleCard {...module} />
            </SwiperSlide>
          ))}
        </Swiper>
      </div>
    </section>
  );
}

export default CourseOverview;
